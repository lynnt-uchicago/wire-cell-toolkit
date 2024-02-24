#include "WireCellImg/PointCloudFacade.h"

using namespace WireCell;
using namespace WireCell::PointCloud;
using namespace WireCell::PointCloud::Facade;
// using WireCell::PointCloud::Dataset;
using namespace WireCell::PointCloud::Tree; // for "Points" node value type
// using WireCell::PointCloud::Tree::named_pointclouds_t;

#include "WireCellUtil/Logging.h"
using spdlog::debug;

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

Blob::Blob(const node_ptr& n)
  : m_node(n.get())
{
    const auto& lpcs = m_node->value.local_pcs();
    const auto& pc_scalar = lpcs.at("scalar");
    // std::cout << "pc_scalar " << dump_ds(pc_scalar) << std::endl;
    charge = pc_scalar.get("charge")->elements<float_t>()[0];
    center_x = pc_scalar.get("center_x")->elements<float_t>()[0];
    center_y = pc_scalar.get("center_y")->elements<float_t>()[0];
    center_z = pc_scalar.get("center_z")->elements<float_t>()[0];
    slice_index_min = pc_scalar.get("slice_index_min")->elements<int_t>()[0];
    slice_index_max = pc_scalar.get("slice_index_max")->elements<int_t>()[0];
    u_wire_index_min = pc_scalar.get("u_wire_index_min")->elements<int_t>()[0];
    u_wire_index_max = pc_scalar.get("u_wire_index_max")->elements<int_t>()[0];
    v_wire_index_min = pc_scalar.get("v_wire_index_min")->elements<int_t>()[0];
    v_wire_index_max = pc_scalar.get("v_wire_index_max")->elements<int_t>()[0];
    w_wire_index_min = pc_scalar.get("w_wire_index_min")->elements<int_t>()[0];
    w_wire_index_max = pc_scalar.get("w_wire_index_max")->elements<int_t>()[0];

}

bool Blob::overlap_fast(const Blob& b, const int offset) const
{
    if (u_wire_index_min > b.u_wire_index_max + offset) return false;
    if (b.u_wire_index_min > u_wire_index_max + offset) return false;
    if (v_wire_index_min > b.v_wire_index_max + offset) return false;
    if (b.v_wire_index_min > v_wire_index_max + offset) return false;
    if (w_wire_index_min > b.w_wire_index_max + offset) return false;
    if (b.w_wire_index_min > w_wire_index_max + offset) return false;
    return true;
}

geo_point_t Blob::center_pos() const {
    return {center_x, center_y, center_z};
}



Cluster::Cluster(const node_ptr& n)
  : m_node(n.get())
{
    // build blobs
    for (const auto& child : m_node->children()) {
        auto blob = std::make_shared<Blob>(child);
        m_blobs.push_back(blob);
        for (int slice_index = blob->slice_index_min; slice_index < blob->slice_index_max; ++slice_index) {
            m_time_blob_map.insert({slice_index, blob});
        }
    }
}

Blob::vector Cluster::is_connected(const Cluster& c, const int offset) const
{
    Blob::vector ret;
    // loop m_time_blob_map
    for (const auto& [time, blob] : m_time_blob_map) {
        // loop c.m_time_blob_map
        auto range = c.m_time_blob_map.equal_range(time);
        for (auto it = range.first; it != range.second; ++it) {
            const auto& cblob = it->second;
            if (blob->overlap_fast(*cblob, offset)) {
                ret.push_back(cblob);
            }
        }
    }
    return ret;
}

geo_point_t Cluster::calc_ave_pos(const geo_point_t& origin, const double dis, const int alg) const {
    spdlog::set_level(spdlog::level::debug); // Set global log level to debug
    /// FIXME: there are many assumptions made, shoud we check these assumptions?
    /// a bit worriying about the speed.
    Scope scope = { "3d", {"x","y","z"} };
    const auto& sv = m_node->value.scoped_view(scope);
    // const auto& spcs = sv.pcs();
    // debug("sv {}", dump_pcs(sv.pcs()));
    const auto& skd = sv.kd();
    auto rad = skd.radius(dis, origin);
    /// FIXME: what if rad is empty?
    if(rad.size() == 0) {
        // raise<ValueError>("empty point cloud");
        return {0,0,0};
    }
    const auto& snodes = sv.nodes();
    std::set<size_t> maj_inds;
    for (size_t pt_ind = 0; pt_ind<rad.size(); ++pt_ind) {
        auto& [pit,dist] = rad[pt_ind];
        const auto [maj_ind,min_ind] = pit.index();
        maj_inds.insert(maj_ind);
    }
    // debug("maj_inds.size() {} ", maj_inds.size());
    geo_point_t ret(0,0,0);
    double total_charge{0};
    for (size_t maj_ind : maj_inds) {
        if (alg == 0) {
            const auto* node = snodes[maj_ind];
            const auto& lpcs = node->value.local_pcs();
            const auto& pc_scalar = lpcs.at("scalar");
            const auto charge = pc_scalar.get("charge")->elements<float_t>()[0];
            const auto center_x = pc_scalar.get("center_x")->elements<float_t>()[0];
            const auto center_y = pc_scalar.get("center_y")->elements<float_t>()[0];
            const auto center_z = pc_scalar.get("center_z")->elements<float_t>()[0];
            // debug("charge {} center {{{} {} {}}}", charge, center_x, center_y, center_z);
            geo_point_t inc(center_x, center_y, center_z);
            inc = inc * charge;
            ret += inc;
            total_charge += charge;
        } else {
            const auto blob = m_blobs[maj_ind];
            const auto charge = blob->charge;
            ret += blob->center_pos() * charge;
            total_charge += charge;
        }
    }
    if (total_charge != 0) {
        ret = ret / total_charge;
    }
    // debug("ret {{{} {} {}}}", ret.x(), ret.y(), ret.z());
    return ret;
}

#include <boost/histogram.hpp>
#include <boost/histogram/algorithm/sum.hpp>
namespace bh = boost::histogram;
namespace bha = boost::histogram::algorithm;


std::pair<double, double> Cluster::hough_transform(const geo_point_t& origin, const double dis, const int alg) const {
    Scope scope = { "3d", {"x","y","z"} };
    const auto& sv = m_node->value.scoped_view(scope);
    const auto& skd = sv.kd();
    auto rad = skd.radius(dis, origin);
    /// FIXME: what if rad is empty?
    if(rad.size() == 0) {
        // raise<ValueError>("empty point cloud");
        return {0,0};
    }
    // const auto& spc = sv.pcs();

    const double pi = 3.141592653589793;
    // axes
    const Vector X(1,0,0);
    const Vector Y(0,1,0);
    const Vector Z(0,0,1);

    if (alg == 0) {
        auto hist = bh::make_histogram(bh::axis::regular<>( 180, -1.0, 1.0 ),
                                       bh::axis::regular<>( 360, -pi, pi ) );

        for (size_t pt_ind = 0; pt_ind<rad.size(); ++pt_ind) {
            auto& [pit,dist] = rad[pt_ind];
            // debug("pt {{{} {} {}}}", pit->at(0), pit->at(1), pit->at(2));
            // auto pt = *pit;
            // debug("pt {{{} {} {}}}", pt[0], pt[1], pt[2]);
            const geo_point_t pt(pit->at(0), pit->at(1), pit->at(2));
            Vector dir = (pt-origin).norm();
            const double cth = Z.dot(dir);
            const double phi = atan2(Y.dot(dir), X.dot(dir));
            hist(cth, phi, bh::weight(1.0));
        }
        
        auto indexed = bh::indexed(hist);
        auto it = std::max_element(indexed.begin(), indexed.end());
        const auto& cell = *it;
        // std::stringstream ss;
        // ss << " maximum: index=[" << cell.index(0) <<","<< cell.index(1) <<"]"
        //    << " cth:[" << cell.bin(0).lower() << "," << cell.bin(0).upper() << "]"
        //    << " phi:[" << cell.bin(1).lower() << "," << cell.bin(1).upper() << "]"
        //    << " value=" << *cell
        //    << " sum=" << bha::sum(hist, bh::coverage::all);
        // spdlog::debug(ss.str());

        // cos(theta), phi
        return {cell.bin(0).center(), cell.bin(1).center()};
    }

    if (alg == 1) {
        auto hist = bh::make_histogram(bh::axis::regular<>( 180, 0, pi ),
                                       bh::axis::regular<>( 360, -pi, pi ) );

        for (size_t pt_ind = 0; pt_ind<rad.size(); ++pt_ind) {
            auto& [pit,dist] = rad[pt_ind];
            const geo_point_t pt(pit->at(0), pit->at(1), pit->at(2));
            Vector dir = (pt-origin).norm();
            const double th = acos(Z.dot(dir));
            const double phi = atan2(Y.dot(dir), X.dot(dir));
            hist(th, phi, bh::weight(1.0));
        }
        auto indexed = bh::indexed(hist);
        auto it = std::max_element(indexed.begin(), indexed.end());
        const auto& cell = *it;
        return {cell.bin(0).center(), cell.bin(1).center()};
    }
    raise<ValueError>("unknown alg %d", alg);
}

geo_point_t Cluster::vhough_transform(const geo_point_t& origin, const double dis, const int alg) const {
    if (alg == 0) {
        const auto [cth, phi] = hough_transform(origin, dis, alg);
        const double sth = sqrt(1-cth*cth);
        return {sth*cos(phi), sth*sin(phi), cth};
    }
    if (alg == 1) {
        const auto [th, phi] = hough_transform(origin, dis, alg);
        return {sin(th)*cos(phi), sin(th)*sin(phi), cos(th)};
    }
    raise<ValueError>("unknown alg %d", alg);
}