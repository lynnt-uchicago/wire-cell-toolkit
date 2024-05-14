#include "WireCellImg/PointCloudFacade.h"

using namespace WireCell;
using namespace WireCell::PointCloud;
using namespace WireCell::PointCloud::Facade;
// using WireCell::PointCloud::Dataset;
using namespace WireCell::PointCloud::Tree; // for "Points" node value type
// using WireCell::PointCloud::Tree::named_pointclouds_t;

#include "WireCellUtil/Logging.h"
using spdlog::debug;

/// unused
#if 0
namespace {
    // helper to dump a dataset
    std::string dump_ds(const WireCell::PointCloud::Dataset& ds) {
        std::stringstream ss;
        for (const auto& key : ds.keys()) {;
            const auto& arr = ds.get(key);
            ss << " {" << key << ":" << arr->dtype() << ":" << arr->shape()[0] << "} ";
            // const auto& arr = ds.get(key)->elements<float>();
            // for(auto elem : arr) {
            //     ss << elem << " ";
            // }
        }
        return ss.str();
    }
    std::string dump_pcs(const ScopedBase::pointclouds_t& pcs) {
        std::stringstream ss;
        for (const auto& pc : pcs) {
            ss << dump_ds(pc) << std::endl;
        }
        return ss.str();
    }
}
#endif

std::ostream& Facade::operator<<(std::ostream& os, const Facade::Blob& blob)
{
    os << "<Blob ["<<(void*)&blob<<"]: nptr="
       << blob.npoints() << " r=" << blob.center_pos()
       << " t=[" << blob.slice_index_min() << "," << blob.slice_index_max() << "]"
       << " u=[" << blob.u_wire_index_min() << "," << blob.u_wire_index_max() << "]"
       << " v=[" << blob.v_wire_index_min() << "," << blob.v_wire_index_max() << "]"
       << " w=[" << blob.w_wire_index_min() << "," << blob.w_wire_index_max() << "]"
       << ">";
    return os;
}

void Blob::on_construct(node_type* node)
{
    this->NaryTree::Facade<points_t>::on_construct(node);

    const auto& lpcs = m_node->value.local_pcs();
    const auto& pc_scalar = lpcs.at("scalar");

    // fixme: transferring these to cache could/should be made lazy.

    // fixme: using a single array of several floats (and etc ints) would avoid
    // many single-entry arrays.

    charge_ = pc_scalar.get("charge")->elements<float_t>()[0];
    center_x_ = pc_scalar.get("center_x")->elements<float_t>()[0];
    center_y_ = pc_scalar.get("center_y")->elements<float_t>()[0];
    center_z_ = pc_scalar.get("center_z")->elements<float_t>()[0];
    npoints_ = pc_scalar.get("npoints")->elements<int_t>()[0];
    slice_index_min_ = pc_scalar.get("slice_index_min")->elements<int_t>()[0];
    slice_index_max_ = pc_scalar.get("slice_index_max")->elements<int_t>()[0];
    u_wire_index_min_ = pc_scalar.get("u_wire_index_min")->elements<int_t>()[0];
    u_wire_index_max_= pc_scalar.get("u_wire_index_max")->elements<int_t>()[0];
    v_wire_index_min_ = pc_scalar.get("v_wire_index_min")->elements<int_t>()[0];
    v_wire_index_max_ = pc_scalar.get("v_wire_index_max")->elements<int_t>()[0];
    w_wire_index_min_ = pc_scalar.get("w_wire_index_min")->elements<int_t>()[0];
    w_wire_index_max_ = pc_scalar.get("w_wire_index_max")->elements<int_t>()[0];
    ///
    ///  MAKE SURE YOU UPDATE doctest_clustering_prototype.cxx if you change the above.
    ///
}

bool Blob::overlap_fast(const Blob& b, const int offset) const
{
    if (u_wire_index_min() > b.u_wire_index_max() + offset) return false;
    if (b.u_wire_index_min() > u_wire_index_max() + offset) return false;
    if (v_wire_index_min() > b.v_wire_index_max() + offset) return false;
    if (b.v_wire_index_min() > v_wire_index_max() + offset) return false;
    if (w_wire_index_min() > b.w_wire_index_max() + offset) return false;
    if (b.w_wire_index_min() > w_wire_index_max() + offset) return false;
    return true;
}

geo_point_t Blob::center_pos() const {
    return {center_x_, center_y_, center_z_};
}

std::ostream& Facade::operator<<(std::ostream& os, const Facade::Cluster& cluster)
{
    os << "<Cluster ["<<(void*)&cluster<<"]:"
       << " npts=" << cluster.npoints()
       << " nblobs=" << cluster.nchildren() << ">";
    return os;
}

const Cluster::time_blob_map_t& Cluster::time_blob_map() const
{
    if (m_time_blob_map.empty()) {
        for (const Blob* blob : children()) {
            m_time_blob_map.insert({blob->slice_index_min(), blob});
        }
    }
    return m_time_blob_map;
}


std::vector<const Blob*> Cluster::is_connected(const Cluster& c, const int offset) const
{
    std::vector<const Blob*> ret;
    for (const auto& [badtime, badblob] : c.time_blob_map()) {
        auto bad_start = badtime;
        auto bad_end = badblob->slice_index_max(); // not inclusive
        for (const auto& [good_start, goodblob] : time_blob_map()) {
            auto good_end = goodblob->slice_index_max(); // not inclusive
            if (good_end < bad_start || good_start >= bad_end) {
                continue;
            }
            if (goodblob->overlap_fast(*badblob, offset)) {
                ret.push_back(goodblob);
            }
        }
    }
    return ret;
}

const Blob* Cluster::get_first_blob() const
{
    if (time_blob_map().empty()) {
        raise<ValueError>("empty cluster has no first blob");
    }
    return time_blob_map().begin()->second;
}

const Blob* Cluster::get_last_blob() const
{
    if (time_blob_map().empty()) {
        raise<ValueError>("empty cluster has no last blob");
    }
    return time_blob_map().rbegin()->second;
}


std::pair<geo_point_t, double>
Cluster::get_closest_point_along_vec(geo_point_t& p_test1, geo_point_t dir,
                                     double test_dis, double dis_step,
                                     double angle_cut, double dis_cut) const{

    geo_point_t p_test;
  
    double min_dis = 1e9;
    double min_dis1 = 1e9;
    geo_point_t min_point = p_test1;
  
    for (int i=0; i!= int(test_dis/dis_step)+1;i++){
        p_test.set(p_test1.x() +
                   dir.x() * i * dis_step, p_test1.y() +
                   dir.y() * i * dis_step, p_test1.z() +
                   dir.z() * i * dis_step);
    
        auto pts = get_closest_point_blob(p_test);
    
        double dis = sqrt(pow(p_test.x() - pts.first.x(),2) +
                          pow(p_test.y() - pts.first.y(),2) +
                          pow(p_test.z() - pts.first.z(),2));
        double dis1 = sqrt(pow(p_test1.x() - pts.first.x(),2) +
                           pow(p_test1.y() - pts.first.y(),2) +
                           pow(p_test1.z() - pts.first.z(),2));

        if (dis < std::min(dis1 * tan(angle_cut/180.*3.1415926),dis_cut)){
            if (dis < min_dis){
                min_dis = dis;
                min_point = pts.first;
                min_dis1 = dis1;
            }
            if (dis < 3*units::cm)
                return std::make_pair(pts.first,dis1);
        }
    }

    return std::make_pair(min_point,min_dis1);
}


int Cluster::npoints() const
{
    if (!m_npoints) {
        const auto& sv = m_node->value.scoped_view(scope);
        m_npoints = sv.npoints();
    }
    return m_npoints;
}


int Cluster::nnearby(const geo_point_t& point, double dis) const
{
    const auto& sv = m_node->value.scoped_view(scope);
    const auto& skd = sv.kd();

    auto rad = skd.radius(dis*dis, point);
    return rad.size();
}


std::pair<int, int> Cluster::ndipole(const geo_point_t& point, const geo_point_t& dir) const
{
    const auto& sv = m_node->value.scoped_view(scope);       // get the kdtree
    const auto& skd = sv.kd();
    const auto& points = skd.points();
    const size_t npoints = points.size();

    int num_p1 = 0;
    int num_p2 = 0;

    for (size_t ind=0; ind<npoints; ++ind) {
        geo_point_t dir1(points[0][ind] - point.x(),
                         points[1][ind] - point.y(),
                         points[2][ind] - point.z());
        if (dir1.dot(dir)>=0){
            ++num_p1;
        }
        else{
            ++num_p2;
        }
    }  

    return std::make_pair(num_p1, num_p2);
}

// std::pair<int, int> Cluster::nprojection(const geo_point_t& point, const geo_point_t& dir, double dis) const
// {
//     const auto& sv = m_node->value.scoped_view(scope);       // get the kdtree
//     const auto& skd = sv.kd();
//     const auto& points = skd.points();

//     int num_p1 = 0;
//     int num_p2 = 0;

//     auto rad = skd.radius(dis*dis, point);
//     for (const auto& [index,_] : rad) {
      
//         geo_point_t dir1(points[0][index] - point.x(),
//                          points[1][index] - point.y(),
//                          points[2][index] - point.z());

//         if (dir1.dot(dir) >= 0) {
//             ++num_p1;
//         }
//         else{
//             ++num_p2;
//         }

//     }

//     return std::make_pair(num_p1, num_p2);
// }



std::map<const Blob*, geo_point_t> Cluster::get_closest_blob(const geo_point_t& point, double radius) const
{
    const auto& sv = m_node->value.scoped_view(scope);       // get the kdtree
    const auto& skd = sv.kd();
    const auto& points = skd.points();

    // This returns L2 - [distance] squared.
    auto results = skd.radius(radius*radius, point);

    std::map<const Blob*, geo_point_t> mcell_point_map; // return
    std::map<const Blob*, double> mcell_dis2_map;       // temp
  

    const auto& blobs = children();

    // fixme: we could reduce the number of calls to map lookups if this loop
    // ends up being a hot spot.
    for (const auto& [index, dist2] : results) {

        geo_point_t p1(points[0][index],
                       points[1][index],
                       points[2][index]);

        const auto blob_index = skd.major_index(index);
        const Blob* blob = blobs[blob_index];

        auto it = mcell_dis2_map.find(blob);
        if (it == mcell_dis2_map.end()) { // first time
            mcell_dis2_map[blob] = dist2;
            mcell_point_map[blob] = p1;
        }
        else {
            if (dist2 < it->second) { // check for yet closer
                it->second = dist2;
                mcell_point_map[blob] = p1;
            }
        }
    }
  
    return mcell_point_map;
}

std::pair<geo_point_t, const Blob* > Cluster::get_closest_point_blob(const geo_point_t& point) const
{
    const auto& sv = m_node->value.scoped_view(scope);       // get the kdtree
    const auto& skd = sv.kd();
    const auto& points = skd.points();

    auto results = skd.knn(1, point);
    if (results.size() == 0) {
        return std::make_pair(geo_point_t(), nullptr);
    }

    const auto& [index, dist2] = results[0];

    const auto blob_index = skd.major_index(index);
    const Blob* blob = children()[blob_index];

    geo_point_t closest(points[0][index],
                        points[1][index],
                        points[2][index]);

    return std::make_pair(closest, blob);
}

geo_point_t Cluster::calc_ave_pos(const geo_point_t& origin, const double dis, const int alg) const
{
    // average position
    geo_point_t out(0,0,0);
    double charge = 0;

    for (auto [blob, pt] : get_closest_blob(origin, dis)) {
        double q = blob->charge();
        if (q==0) q=1;
        out += q*blob->center_pos();
        charge += q;
    }
    
    if (charge != 0) {
        out = out / charge;
    }

    return out;
}

#include <boost/histogram.hpp>
#include <boost/histogram/algorithm/sum.hpp>
namespace bh = boost::histogram;
namespace bha = boost::histogram::algorithm;


// Example parameter calculating functions used by directional hough
// transforms.
static
double theta_angle(const Vector& dir) {
    const Vector Z(0,0,1);
    return acos(Z.dot(dir)); 
}
static
double theta_cosine(const Vector& dir) { 
    const Vector Z(0,0,1);
    return Z.dot(dir); 
}
static
double phi_angle(const Vector& dir) {
    const Vector X(1,0,0);
    const Vector Y(0,1,0);
    return atan2(Y.dot(dir), X.dot(dir));
}




std::pair<double, double>
Cluster::hough_transform(const geo_point_t& origin, const double dis,
                         HoughParamSpace param_space) const
{

    // Scope scope = { "3d", {"x","y","z"} };
    const auto& sv = m_node->value.scoped_view(scope);
    const auto& skd = sv.kd();
    const auto& points = skd.points();

    auto results = skd.radius(dis*dis, origin);
    /// FIXME: what if results is empty?
    if(results.size() == 0) {
        // raise<ValueError>("empty point cloud");
        // (bv) why not raise?  we do below.
        return {0,0};
    }
    constexpr double pi = 3.141592653589793;

    using direction_parameter_function_f = std::function<double(const Vector& dir)>;

    // Parameter axis 1 is some measure of theta angle (angle or cosine)
    const int nbins1 = 180;
    // param_space == costh_phi
    direction_parameter_function_f theta_param = theta_cosine;
    double min1=-1.0, max1=1.0;
    if ( param_space == HoughParamSpace::theta_phi ) {
        theta_param = theta_angle;
        min1 = 0;
        max1 = pi;
    }

    // Parameter axis 2 is only supported by phi angle
    const int nbins2 = 360;
    const double min2 = -pi;
    const double max2 = +pi;
    direction_parameter_function_f phi_param = phi_angle;

    auto hist = bh::make_histogram(bh::axis::regular<>( nbins1, min1, max1 ),
                                   bh::axis::regular<>( nbins2, min2, max2 ) );

    const auto& blobs = children();

    for (const auto& [index, _] : results) {

        const auto blob_index = skd.major_index(index);
        const auto* blob = blobs[blob_index];
        auto charge = blob->charge();
        // protection against the charge=0 case ...
        if (charge == 0) charge = 1;
        if (charge <= 0) continue;
            
        const auto npoints = blob->npoints();

        geo_point_t pt(points[0][index],
                       points[1][index],
                       points[2][index]);

        Vector dir = (pt-origin).norm();

        const double p1 = theta_param(dir);
        const double p2 = phi_param(dir);
        hist(p1, p2, bh::weight(charge/npoints));
    }
        
    auto indexed = bh::indexed(hist);
    auto it = std::max_element(indexed.begin(), indexed.end());
    const auto& cell = *it;
    return {cell.bin(0).center(), cell.bin(1).center()};
}


geo_point_t Cluster::vhough_transform(const geo_point_t& origin, const double dis,
                                      HoughParamSpace param_space) const
{
    if ( param_space == HoughParamSpace::theta_phi ) {
        const auto [th, phi] = hough_transform(origin, dis, param_space);
        return {sin(th)*cos(phi), sin(th)*sin(phi), cos(th)};
    }
    // costh_phi
    const auto [cth, phi] = hough_transform(origin, dis, param_space);
    const double sth = sqrt(1-cth*cth);
    return {sth*cos(phi), sth*sin(phi), cth};
}


std::tuple<int, int, int, int> Cluster::get_uvwt_min() const
{
    std::tuple<int, int, int, int> ret;
    bool first = true;
    
    for (const auto* blob : children()) {
        const int u = blob->u_wire_index_min();
        const int v = blob->v_wire_index_min();
        const int w = blob->w_wire_index_min();
        const int t = blob->slice_index_min();

        if (first) {
            ret = {u,v,w,t};
            continue;
        }
        get<0>(ret) = std::min(get<0>(ret), u);
        get<1>(ret) = std::min(get<1>(ret), v);
        get<2>(ret) = std::min(get<2>(ret), w);
        get<3>(ret) = std::min(get<3>(ret), t);
    }
    return ret;
}
std::tuple<int, int, int, int> Cluster::get_uvwt_max() const
{
    std::tuple<int, int, int, int> ret;
    bool first = true;
    
    for (const auto* blob : children()) {
        const int u = blob->u_wire_index_max();
        const int v = blob->v_wire_index_max();
        const int w = blob->w_wire_index_max();
        const int t = blob->slice_index_max();

        if (first) {
            ret = {u,v,w,t};
            continue;
        }
        get<0>(ret) = std::max(get<0>(ret), u);
        get<1>(ret) = std::max(get<1>(ret), v);
        get<2>(ret) = std::max(get<2>(ret), w);
        get<3>(ret) = std::max(get<3>(ret), t);
    }
    return ret;
}

// FIXME: Is this actually correct?  It does not return "ranges" but rather the
// number of unique wires/ticks in the cluster.  A sparse but large cluster will
// be "smaller" than a small but dense cluster.
std::tuple<int, int, int, int> Cluster::get_uvwt_range() const
{
    std::set<int> u_set;
    std::set<int> v_set;
    std::set<int> w_set;
    std::set<int> t_set;
    for (const auto* blob : children()) {
        for(int i = blob->u_wire_index_min(); i < blob->u_wire_index_max(); ++i) {
            u_set.insert(i);
        }
        for(int i = blob->v_wire_index_min(); i < blob->v_wire_index_max(); ++i) {
            v_set.insert(i);
        }
        for(int i = blob->w_wire_index_min(); i < blob->w_wire_index_max(); ++i) {
            w_set.insert(i);
        }
        for(int i = blob->slice_index_min(); i < blob->slice_index_max(); ++i) {
            t_set.insert(i);
        }
    }
    return {u_set.size(), v_set.size(), w_set.size(), t_set.size()};
}


double Cluster::get_length(const TPCParams& tp) const
{
    if (m_length == 0) {        // invalidates when a new node is set
        const auto [u, v, w, t] = get_uvwt_range();
        const double pu = u*tp.pitch_u;
        const double pv = v*tp.pitch_v;
        const double pw = w*tp.pitch_w;
        const double pt = t*tp.tick_drift;
        m_length = std::sqrt(2./3.*(pu*pu + pv*pv + pw*pw) + pt*pt);
    }
    return m_length;
}


std::pair<geo_point_t, geo_point_t> Cluster::get_highest_lowest_points(size_t axis) const
{
    const auto& sv = m_node->value.scoped_view(scope);
    const auto& skd = sv.kd();
    const auto& points = skd.points();
    const size_t npoints = points.size();

    geo_point_t lowest_point, highest_point;

    for (size_t ind=0; ind<npoints; ++ind) {
        geo_point_t pt(points[0][ind],
                       points[1][ind],
                       points[2][ind]);
        if (!ind) {
            lowest_point = highest_point = pt;
            continue;
        }
        if (pt[axis] > highest_point[axis]) {
            highest_point = pt;
        }
        if (pt[axis] < lowest_point[axis]) {
            lowest_point = pt;
        }
    }

    return std::make_pair(highest_point, lowest_point);
}


std::pair<geo_point_t, geo_point_t> Cluster::get_earliest_latest_points() const
{
    auto backwards = get_highest_lowest_points(0);
    return std::make_pair(backwards.second, backwards.first);
}


std::ostream& Facade::operator<<(std::ostream& os, const Facade::Grouping& grouping)
{
    os << "<Grouping ["<<(void*)&grouping<<"]:"
       << " nclusters=" << grouping.nchildren() << ">";
    return os;
}


std::string Facade::dump(const Facade::Grouping& grouping, int level)
{
    std::stringstream ss;

    ss << grouping;
    if (level == 0) {
        return ss.str();
    }
    ss << "\n";
    size_t nc=0;
    for (const auto* cluster : grouping.children()) {
        ss << nc++ << "\t" << *cluster << "\n";
        if (level == 1) {
            continue;
        }
        size_t nb = 0;
        for (const auto* blob : cluster->children()) {
            ss << nb++ << "\t\t" << *blob << "\n";
        }
    }
    return ss.str();
}


// Local Variables:
// mode: c++
// c-basic-offset: 4
// End:
