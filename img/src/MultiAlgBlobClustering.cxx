#include "WireCellImg/MultiAlgBlobClustering.h"
#include "WireCellImg/PointCloudFacade.h"
#include "WireCellUtil/NamedFactory.h"
#include "WireCellUtil/Units.h"
#include "WireCellUtil/Persist.h"
#include "WireCellAux/TensorDMpointtree.h"
#include "WireCellAux/TensorDMdataset.h"
#include "WireCellAux/TensorDMcommon.h"
#include "WireCellAux/SimpleTensorSet.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>

#include <fstream>

WIRECELL_FACTORY(MultiAlgBlobClustering, WireCell::Img::MultiAlgBlobClustering, WireCell::INamed,
                 WireCell::ITensorSetFilter, WireCell::IConfigurable)

using namespace WireCell;
using namespace WireCell::Img;
using namespace WireCell::Aux;
using namespace WireCell::Aux::TensorDM;
using namespace WireCell::PointCloud::Facade;
using namespace WireCell::PointCloud::Tree;

MultiAlgBlobClustering::MultiAlgBlobClustering()
  : Aux::Logger("MultiAlgBlobClustering", "img")
{
}

void MultiAlgBlobClustering::configure(const WireCell::Configuration& cfg)
{
    m_inpath = get(cfg, "inpath", m_inpath);
    m_outpath = get(cfg, "outpath", m_outpath);
    m_bee_dir = get(cfg, "bee_dir", m_bee_dir);

    m_dead_live_overlap_offset = get(cfg, "dead_live_overlap_offset", m_dead_live_overlap_offset);
}

WireCell::Configuration MultiAlgBlobClustering::default_configuration() const
{
    Configuration cfg;
    cfg["inpath"] = m_inpath;
    cfg["outpath"] = m_outpath;
    cfg["bee_dir"] = m_bee_dir;

    cfg["dead_live_overlap_offset"] = m_dead_live_overlap_offset;
    return cfg;
}

namespace {
    void dump_bee(const Points::node_t& root, const std::string& fn)
    {
        using spdlog::debug;
        using WireCell::PointCloud::Facade::float_t;
        using WireCell::PointCloud::Facade::int_t;

        Configuration bee;
        bee["runNo"] = 0;
        bee["subRunNo"] = 0;
        bee["eventNo"] = 0;
        bee["geom"] = "uboone";
        bee["type"] = "cluster";

        std::vector<float_t> x;
        std::vector<float_t> y;
        std::vector<float_t> z;
        std::vector<float_t> q;
        std::vector<int_t> cluster_id;
        int_t cid = 0;
        for (const auto& cnode : root.children()) {  // this is a loop through all clusters ...
            Scope scope = {"3d", {"x", "y", "z"}};
            const auto& sv = cnode->value.scoped_view(scope);

            const auto& spcs = sv.pcs();  // spcs 'contains' all blobs in this cluster ...
            // int npoints = 0;

            for (const auto& spc : spcs) {  // each little 3D pc --> (blobs)   spc represents x,y,z in a blob
                if (spc.get().get("x") == nullptr) {
                    debug("No x in point cloud, skip");
                    continue;
                }

                // assume others exist
                const auto& x_ = spc.get().get("x")->elements<float_t>();
                const auto& y_ = spc.get().get("y")->elements<float_t>();
                const auto& z_ = spc.get().get("z")->elements<float_t>();
                const size_t n = x_.size();
                x.insert(x.end(), x_.begin(), x_.end());  // Append x_ to x
                y.insert(y.end(), y_.begin(), y_.end());
                z.insert(z.end(), z_.begin(), z_.end());
                q.insert(q.end(), n, 1.0);
                cluster_id.insert(cluster_id.end(), n, cid);
                // npoints += n;
            }

            // spc.kd() // kdtree ...
            // const auto& skd = sv.kd();
            // std::cout << "xin6: " << sv.npoints() << " " << npoints << " " << spcs.size() << " " <<
            // skd.points().size() << std::endl;

            ++cid;
        }

        Json::Value json_x(Json::arrayValue);
        for (const auto& val : x) {
            json_x.append(val / units::cm);
        }
        bee["x"] = json_x;

        Json::Value json_y(Json::arrayValue);
        for (const auto& val : y) {
            json_y.append(val / units::cm);
        }
        bee["y"] = json_y;

        Json::Value json_z(Json::arrayValue);
        for (const auto& val : z) {
            json_z.append(val / units::cm);
        }
        bee["z"] = json_z;

        Json::Value json_q(Json::arrayValue);
        for (const auto& val : q) {
            json_q.append(val);
        }
        bee["q"] = json_q;

        Json::Value json_cluster_id(Json::arrayValue);
        for (const auto& val : cluster_id) {
            json_cluster_id.append(val);
        }
        bee["cluster_id"] = json_cluster_id;

        // Write cfg to file
        std::ofstream file(fn);
        if (file.is_open()) {
            Json::StreamWriterBuilder writer;
            writer["indentation"] = "    ";
            std::unique_ptr<Json::StreamWriter> jsonWriter(writer.newStreamWriter());
            jsonWriter->write(bee, &file);
            file.close();
        }
        else {
            raise<ValueError>("Failed to open file: " + fn);
        }
    }

    struct Point2D {
        Point2D(float x, float y)
          : x(x)
          , y(y)
        {
        }
        float x;
        float y;
    };

    bool anglular_less(const Point2D& a, const Point2D& b, const Point2D& center)
    {
        if (a.x - center.x >= 0 && b.x - center.x < 0) return true;
        if (a.x - center.x < 0 && b.x - center.x >= 0) return false;
        if (a.x - center.x == 0 && b.x - center.x == 0) {
            if (a.y - center.y >= 0 || b.y - center.y >= 0) return a.y > b.y;
            return b.y > a.y;
        }

        // compute the cross product of vectors (center -> a) x (center -> b)
        int det = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
        if (det < 0) return true;
        if (det > 0) return false;

        // points a and b are on the same line from the center
        // check which point is closer to the center
        int d1 = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
        int d2 = (b.x - center.x) * (b.x - center.x) + (b.y - center.y) * (b.y - center.y);
        return d1 > d2;
    }

    std::vector<Point2D> sort_angular(const std::vector<Point2D>& points)
    {
        if (points.size() < 3) {
            return points;
        }
        // Calculate the center of the points
        float sumX = 0.0;
        float sumY = 0.0;
        for (const auto& point : points) {
            sumX += point.x;
            sumY += point.y;
        }
        Point2D center(sumX / points.size(), sumY / points.size());

        std::vector<Point2D> sorted = points;
        std::sort(sorted.begin(), sorted.end(),
                  [&](const Point2D& a, const Point2D& b) { return anglular_less(a, b, center); });
        return sorted;
    }

    void dumpe_deadarea(const Points::node_t& root, const std::string& fn)
    {
        using WireCell::PointCloud::Facade::float_t;
        using WireCell::PointCloud::Facade::int_t;

        // Convert stringstream to JSON array
        Json::Value jdead;
        Json::Reader reader;
        for (const auto& cnode : root.children()) {
            for (const auto& bnode : cnode->children()) {
                const auto& lpcs = bnode->value.local_pcs();
                const auto& pc_scalar = lpcs.at("corner");
                const auto& y = pc_scalar.get("y")->elements<float_t>();
                const auto& z = pc_scalar.get("z")->elements<float_t>();
                std::vector<Point2D> points;
                for (size_t i = 0; i < y.size(); ++i) {
                    points.push_back({y[i], z[i]});
                }
                auto sorted = sort_angular(points);
                Json::Value jarea(Json::arrayValue);
                for (const auto& point : sorted) {
                    Json::Value jpoint(Json::arrayValue);
                    jpoint.append(point.x / units::cm);
                    jpoint.append(point.y / units::cm);
                    jarea.append(jpoint);
                }
                jdead.append(jarea);
            }
        }

        // Output jsonArray to file
        std::ofstream file(fn);
        if (file.is_open()) {
            file << jdead.toStyledString();
            file.close();
        }
        else {
            std::cout << "Failed to open file: " << fn << std::endl;
        }
    }
}  // namespace

namespace {
    using spdlog::debug;
    void clustering_live_dead(Points::node_ptr& root_live,                                   // in/out
                              const Points::node_ptr& root_dead,                             // in
                              std::map<const Cluster::pointer, double>& cluster_length_map,  // in/out
                              std::set<Cluster::pointer>& cluster_connected_dead,            // in/out
                              const TPCParams& tp,                                           // common params
                              const int dead_live_overlap_offset                             // specific params
    )
    {
        std::unordered_map<std::string, std::chrono::milliseconds> timers;
        auto start = std::chrono::high_resolution_clock::now();
        Cluster::vector live_clusters;
        for (const auto& cnode : root_live->children()) {
            live_clusters.push_back(std::make_shared<Cluster>(cnode));
        }
        Cluster::vector dead_clusters;
        for (const auto& cnode : root_dead->children()) {
            dead_clusters.push_back(std::make_shared<Cluster>(cnode));
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        timers["make_facade"] += duration;
        debug("make_facade {} live {} dead {} ms", live_clusters.size(), dead_clusters.size(),
              timers["make_facade"].count());

        std::set<std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> > cluster_to_be_deleted;

        // loop over all the clusters, and calculate length ...
        for (size_t ilive = 0; ilive < live_clusters.size(); ++ilive) {
            const auto& live = live_clusters[ilive];
            cluster_length_map[live] = live->get_length(tp);
            // std::cout << ilive << " xin " << live->get_length(tp)/units::cm << std::endl;
        }

        // form dead -> lives map
        // start = std::chrono::high_resolution_clock::now();
        // std::unordered_map<Cluster::pointer, Cluster::vector> dead2lives;
        // for (size_t idead = 0; idead < dead_clusters.size(); ++idead) {
        //     const auto& dead = dead_clusters[idead];
        //     Cluster::vector lives;
        //     for (const auto& live : live_clusters) {
        //         if (live->is_connected(*dead, m_dead_live_overlap_offset).size()) {
        //             lives.push_back(live);
        //         }
        //     }
        //     dead2lives[dead] = std::move(lives);
        //     if (dead2lives[dead].size() > 1) {
        //         debug("dead2lives-map {} {} ", idead, dead2lives[dead].size());
        //     }
        // }
        // end = std::chrono::high_resolution_clock::now();
        // duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        // timers["dead2lives-map"] += duration;
        // debug("dead2lives-map {} ms", timers["dead2lives-map"].count());

        // form map between live and dead clusters ...
        std::map<const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster>, Cluster::vector>
            dead_live_cluster_mapping;
        std::map<const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster>, std::vector<Blob::vector> >
            dead_live_mcells_mapping;
        for (size_t idead = 0; idead < dead_clusters.size(); ++idead) {
            const auto& dead = dead_clusters[idead];
            for (size_t ilive = 0; ilive < live_clusters.size(); ++ilive) {
                // const auto& live = live_clusters[ilive];
                const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> live = live_clusters[ilive];

                Blob::vector blobs = live->is_connected(*dead, dead_live_overlap_offset);
                //
                if (blobs.size() > 0) {
                    //	  if (dead_live_cluster_mapping.find(dead) == dead_live_cluster_mapping.end()){
                    dead_live_cluster_mapping[dead].push_back(live);
                    dead_live_mcells_mapping[dead].push_back(blobs);
                    //}
                }
            }
        }

        // prepare a graph ...
        typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, int> Graph;
        Graph g;
        std::unordered_map<int, int> ilive2desc;  // added live index to graph descriptor
        std::map<const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster>, int> map_cluster_index;
        for (size_t ilive = 0; ilive < live_clusters.size(); ++ilive) {
            const auto& live = live_clusters[ilive];
            map_cluster_index[live] = ilive;
            ilive2desc[ilive] = boost::add_vertex(ilive, g);
        }

        std::set<std::pair<std::shared_ptr<const WireCell::PointCloud::Facade::Cluster>,
                           std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> > >
            tested_pairs;

        // start to form edges ...
        for (auto it = dead_live_cluster_mapping.begin(); it != dead_live_cluster_mapping.end(); it++) {
            const auto& the_dead_cluster = (*it).first;
            const auto& connected_live_clusters = (*it).second;
            const auto& connected_live_mcells = dead_live_mcells_mapping[the_dead_cluster];

            if (connected_live_clusters.size() > 1) {
                std::cout << "xin " << connected_live_clusters.size() << " " << connected_live_mcells.size()
                          << std::endl;

                for (size_t i = 0; i != connected_live_clusters.size(); i++) {
                    const auto& cluster_1 = connected_live_clusters.at(i);
                    const auto& blobs_1 = connected_live_mcells.at(i);
                    cluster_connected_dead.insert(cluster_1);

                    for (size_t j = i + 1; j < connected_live_clusters.size(); j++) {
                        const auto& cluster_2 = connected_live_clusters.at(j);
                        const auto& blobs_2 = connected_live_mcells.at(j);

                        std::cout << "xin1 " << i << " " << j << " " << blobs_1.size() << " " << blobs_2.size()
                                  << std::endl;

                        if (tested_pairs.find(std::make_pair(cluster_1, cluster_2)) == tested_pairs.end()) {
                            tested_pairs.insert(std::make_pair(cluster_1, cluster_2));
                            tested_pairs.insert(std::make_pair(cluster_2, cluster_1));

                            bool flag_merge = false;

                            std::shared_ptr<const WireCell::PointCloud::Facade::Blob> prev_mcell1 = 0;
                            std::shared_ptr<const WireCell::PointCloud::Facade::Blob> prev_mcell2 = 0;
                            std::shared_ptr<const WireCell::PointCloud::Facade::Blob> mcell1 = blobs_1.at(0);
                            std::shared_ptr<const WireCell::PointCloud::Facade::Blob> mcell2 = 0;

                            geo_point_t p1 = mcell1->center_pos();
                            std::tie(p1, mcell1) = cluster_1->get_closest_point(p1);
                            //	      p1 = temp_pair.first;
                            //	      mcell1 = temp_pair.second;
                            geo_point_t p2(0, 0, 0);
                            while (mcell1 != prev_mcell1 || mcell2 != prev_mcell2) {
                                prev_mcell1 = mcell1;
                                prev_mcell2 = mcell2;

                                std::tie(p2, mcell2) = cluster_2->get_closest_point(p1);
                                std::tie(p1, mcell1) = cluster_1->get_closest_point(p2);
                            }
                            geo_point_t diff = p1 - p2;
                            double dis = diff.magnitude();

                            std::cout << "xin3 " << dis / units::cm << std::endl;

                            if (dis < 60 * units::cm) {
                                double length_1 = cluster_length_map[cluster_1];
                                double length_2 = cluster_length_map[cluster_2];

                                geo_point_t mcell1_center = cluster_1->calc_ave_pos(p1, 5 * units::cm, 1);
                                geo_point_t dir1 = cluster_1->vhough_transform(mcell1_center, 30 * units::cm, 1);

                                geo_point_t mcell2_center = cluster_2->calc_ave_pos(p2, 5 * units::cm, 1);
                                geo_point_t dir3 = cluster_2->vhough_transform(mcell2_center, 30 * units::cm, 1);

                                geo_point_t dir2 = mcell2_center - mcell1_center;
                                geo_point_t dir4 = mcell1_center - mcell2_center;

                                double angle_diff1 = (3.1415926 - dir1.angle(dir2)) / 3.1415926 * 180.;  // 1 to 2
                                double angle_diff2 = (3.1415926 - dir3.angle(dir4)) / 3.1415926 * 180.;  // 2 to 1
                                double angle_diff3 = (3.1415926 - dir1.angle(dir3)) / 3.1415926 * 180.;  // 1 to 2

                                if (length_1 > 100 * units::cm && length_2 > 100 * units::cm) {
                                    std::cout << "xin4 " << length_1 / units::cm << " " << length_2 / units::cm
                                              << std::endl;
                                    std::cout << "xin5 " << p1 << " " << p2 << " " << mcell1_center << " "
                                              << mcell2_center << " " << dir1 << " " << dir3 << " " << angle_diff1
                                              << " " << angle_diff2 << " " << angle_diff3 << std::endl;
                                }

                                bool flag_para = false;

                                double angle1, angle2, angle3;
                                if (!flag_merge) {
                                    geo_point_t drift_dir(1, 0, 0);  // assuming the drift direction is along X ...
                                    angle1 = dir1.angle(drift_dir);
                                    angle2 = dir2.angle(drift_dir);
                                    angle3 = dir3.angle(drift_dir);

                                    if (fabs(angle1 - 3.1415926 / 2.) < 5 / 180. * 3.1415926 &&
                                        fabs(angle2 - 3.1415926 / 2.) < 5 / 180. * 3.1415926 &&
                                        fabs(angle3 - 3.1415926 / 2.) < 5 / 180. * 3.1415926) {
                                        if (dis < 10 * units::cm)  // if very parallel and close, merge any way
                                            flag_merge = true;
                                    }

                                    if (fabs(angle2 - 3.1415926 / 2.) < 7.5 / 180. * 3.1415926 &&
                                        (fabs(angle1 - 3.1415926 / 2.) < 7.5 / 180. * 3.1415926 ||
                                         fabs(angle3 - 3.1415926 / 2.) < 7.5 / 180. * 3.1415926) &&
                                        fabs(angle1 - 3.1415926 / 2.) + fabs(angle2 - 3.1415926 / 2.) +
                                                fabs(angle3 - 3.1415926 / 2.) <
                                            25 / 180. * 3.1415926) {
                                        flag_para = true;

                                        if (WireCell::PointCloud::Facade::is_angle_consistent(
                                                dir1, dir2, false, 15, tp.angle_u, tp.angle_v, tp.angle_w, 3) &&
                                            WireCell::PointCloud::Facade::is_angle_consistent(
                                                dir3, dir2, true, 15, tp.angle_u, tp.angle_v, tp.angle_w, 3))
                                            flag_merge = true;
                                    }
                                    else {
                                        bool flag_const1 = WireCell::PointCloud::Facade::is_angle_consistent(
                                            dir1, dir2, false, 10, tp.angle_u, tp.angle_v, tp.angle_w, 2);
                                        bool flag_const2 = WireCell::PointCloud::Facade::is_angle_consistent(
                                            dir3, dir2, true, 10, tp.angle_u, tp.angle_v, tp.angle_w, 2);

                                        if (flag_const1 && flag_const2) {
                                            flag_merge = true;
                                        }
                                        else if (flag_const1 && length_2 < 6 * units::cm && length_1 > 15 * units::cm) {
                                            if (WireCell::PointCloud::Facade::is_angle_consistent(
                                                    dir1, dir2, false, 5, tp.angle_u, tp.angle_v, tp.angle_w, 3))
                                                flag_merge = true;
                                        }
                                        else if (flag_const2 && length_1 < 6 * units::cm && length_2 > 15 * units::cm) {
                                            if (WireCell::PointCloud::Facade::is_angle_consistent(
                                                    dir3, dir2, true, 5, tp.angle_u, tp.angle_v, tp.angle_w, 3))
                                                flag_merge = true;
                                        }
                                    }
                                }

                                if (!flag_merge) {
                                    if (length_1 <= 12 * units::cm && length_2 <= 12 * units::cm) {
                                        // both are short
                                        if ((dis <= 3 * units::cm) &&
                                                ((angle_diff1 <= 45 || angle_diff2 <= 45) && (angle_diff3 < 60) ||
                                                 (flag_para && (angle_diff1 <= 90 || angle_diff2 <= 90) &&
                                                  angle_diff3 < 120)) ||
                                            (dis <= 5 * units::cm) && (angle_diff1 <= 30 || angle_diff2 <= 30) &&
                                                angle_diff3 < 45 ||
                                            (dis <= 15 * units::cm) && (angle_diff1 <= 15 || angle_diff2 <= 15) &&
                                                angle_diff3 < 20 ||
                                            (dis <= 60 * units::cm) && (angle_diff1 < 5 || angle_diff2 < 5) &&
                                                angle_diff3 < 10) {
                                            flag_merge = true;
                                        }
                                    }
                                    else if (length_1 > 12 * units::cm && length_2 <= 12 * units::cm) {
                                        // one is short
                                        if ((dis <= 3 * units::cm) &&
                                                ((angle_diff1 <= 45 || angle_diff2 <= 45) && (angle_diff3 < 60) ||
                                                 (flag_para && (angle_diff1 <= 90 || angle_diff2 <= 90) &&
                                                  angle_diff3 < 120)) ||
                                            dis <= 5 * units::cm && angle_diff1 <= 30 && angle_diff3 < 60 ||
                                            dis <= 15 * units::cm && (angle_diff1 <= 20) && angle_diff3 < 40 ||
                                            (angle_diff1 < 10 && dis <= 60 * units::cm && angle_diff3 < 15))
                                            flag_merge = true;
                                    }
                                    else if (length_2 > 12 * units::cm && length_1 <= 12 * units::cm) {
                                        // one is short
                                        if ((dis <= 3 * units::cm) &&
                                                ((angle_diff2 <= 45 || angle_diff2 <= 45) && (angle_diff3 < 60) ||
                                                 (flag_para && (angle_diff1 <= 90 || angle_diff2 <= 90) &&
                                                  angle_diff3 < 120)) ||
                                            dis <= 5 * units::cm && angle_diff2 <= 30 && angle_diff3 < 60 ||
                                            dis <= 15 * units::cm && (angle_diff2 <= 20) && angle_diff3 < 40 ||
                                            (angle_diff2 < 10 && dis <= 60 * units::cm && angle_diff3 < 15))
                                            flag_merge = true;
                                    }
                                    else {
                                        // both are long
                                        if ((dis <= 3 * units::cm) &&
                                                ((angle_diff1 <= 45 || angle_diff2 <= 45) && (angle_diff3 < 60) ||
                                                 (flag_para && (angle_diff1 <= 90 || angle_diff2 <= 90) &&
                                                  angle_diff3 < 120)) ||
                                            dis <= 5 * units::cm && (angle_diff1 <= 30 || angle_diff2 <= 30) &&
                                                angle_diff3 < 45 ||
                                            (dis <= 15 * units::cm) && (angle_diff1 <= 20 || angle_diff2 <= 20) &&
                                                angle_diff3 < 30 ||
                                            (angle_diff1 < 10 || angle_diff2 < 10) && (dis <= 60 * units::cm) &&
                                                angle_diff3 < 15)
                                            flag_merge = true;
                                    }
                                }
                            }

                            //	      flag_merge = true;
                            std::cout << "xin2: " << flag_merge << std::endl;

                            if (flag_merge) {
                                boost::add_edge(ilive2desc[map_cluster_index[cluster_1]],
                                                ilive2desc[map_cluster_index[cluster_2]], g);
                                cluster_to_be_deleted.insert(cluster_1);
                                cluster_to_be_deleted.insert(cluster_2);
                            }
                        }
                    }
                }
            }
        }

        for (auto it = cluster_to_be_deleted.begin(); it != cluster_to_be_deleted.end(); it++) {
            cluster_length_map.erase(*it);
            cluster_connected_dead.erase(*it);
        }

        // // from dead -> lives graph
        // start = std::chrono::high_resolution_clock::now();
        // // dead: negative, live: positive
        // for (size_t idead = 0; idead < dead_clusters.size(); ++idead) {
        //     const auto& dead = dead_clusters[idead];
        //     const auto ddesc = boost::add_vertex(-idead, g);
        //     for (size_t ilive = 0; ilive < live_clusters.size(); ++ilive) {
        //         const auto& live = live_clusters[ilive];
        //         // insert live to graph if not already
        //         if (ilive2desc.find(ilive) == ilive2desc.end()) {
        //             ilive2desc[ilive] = boost::add_vertex(ilive, g);
        //         }
        //         if (live->is_connected(*dead, m_dead_live_overlap_offset).size()) {
        //             boost::add_edge(ddesc, ilive2desc[ilive], g);
        //         }
        //     }
        //     if (boost::out_degree(ddesc, g) > 1) {
        //         debug("dead2lives-graph {} {} {} {} ", idead, ddesc, g[ddesc], boost::out_degree(ddesc, g));
        //     }
        // }
        // end = std::chrono::high_resolution_clock::now();
        // duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        // timers["dead2lives-graph"] += duration;
        // debug("dead2lives-graph {} ms", timers["dead2lives-graph"].count());

        // Make new live node tree
        Points::node_ptr root_live_new = std::make_unique<Points::node_t>();
        debug("root_live {} root_live_new {}", root_live->children().size(), root_live_new->children().size());
        // std::unordered_set<Cluster::pointer> need_merging;
        // for (const auto& [dead, lives] : dead2lives) {
        //     if (lives.size() < 2) {
        //         continue;
        //     }
        //     debug("dead2lives size for dead cluster: {}", lives.size());
        //     need_merging.insert(lives.begin(), lives.end());
        //     auto cnode = root_live_new->insert(std::move(std::make_unique<Points::node_t>()));
        //     for (const auto& live : lives) {
        //         for (const auto& blob : live->m_blobs) {
        //             // this also removes blob node from root_live
        //             cnode->insert(blob->m_node);
        //         }
        //         // manually remove the cnode from root_live
        //         root_live->remove(live->m_node);
        //     }
        // }
        // debug("need_merging size: {}", need_merging.size());

        std::unordered_map<int, int> desc2id;
        std::unordered_map<int, std::set<int> > id2desc;
        int num_components = boost::connected_components(g, boost::make_assoc_property_map(desc2id));
        for (const auto& [desc, id] : desc2id) {
            id2desc[id].insert(desc);
        }
        debug("id2desc size: {}", id2desc.size());
        for (const auto& [id, descs] : id2desc) {
            if (descs.size() < 3) {
                continue;
            }
            debug("id {} descs size: {}", id, descs.size());

            auto cnode1 = std::make_unique<Points::node_t>();
            for (const auto& desc : descs) {
                const int idx = g[desc];
                if (idx < 0) {  // no need anymore ...
                    continue;
                }
                const auto& live = live_clusters[idx];
                for (const auto& blob : live->m_blobs) {
                    // this also removes blob node from root_live
                    cnode1->insert(blob->m_node);
                }
                // manually remove the cnode from root_live
                root_live->remove(live->m_node);
            }

            // new cluster information (need Haiwang to take a look at Facade ...)
            auto new_cluster = std::make_shared<Cluster>(cnode1);
            auto cnode = root_live_new->insert(std::move(cnode1));
            cluster_length_map[new_cluster] = new_cluster->get_length(tp);
            cluster_connected_dead.insert(new_cluster);
        }
        debug("root_live {} root_live_new {}", root_live->children().size(), root_live_new->children().size());

        // move remaining live clusters to new root
        for (auto& cnode : root_live->children()) {
            // this will NOT remove cnode from root_live, but set it to nullptr
            root_live_new->insert(std::move(cnode));
        }
        debug("root_live {} root_live_new {}", root_live->children().size(), root_live_new->children().size());
        // replace old with new
        root_live = std::move(root_live_new);
    }
}  // namespace

bool MultiAlgBlobClustering::operator()(const input_pointer& ints, output_pointer& outts)
{
    outts = nullptr;
    if (!ints) {
        log->debug("EOS at call {}", m_count++);
        return true;
    }

    const int ident = ints->ident();
    std::string inpath = m_inpath;
    if (inpath.find("%") != std::string::npos) {
        inpath = String::format(inpath, ident);
    }

    const auto& intens = *ints->tensors();
    log->debug("Input {} tensors", intens.size());
    auto start = std::chrono::high_resolution_clock::now();
    auto root_live = std::move(as_pctree(intens, inpath + "/live"));
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    log->debug("as_pctree for {} took {} ms", inpath + "/live", duration.count());
    if (!root_live) {
        log->error("Failed to get point cloud tree from \"{}\"", inpath);
        return false;
    }
    log->debug("Got pctree with {} children", root_live->children().size());

    start = std::chrono::high_resolution_clock::now();
    const auto& root_dead = as_pctree(intens, inpath + "/dead");
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    log->debug("as_pctree for {} took {} ms", inpath + "/dead", duration.count());
    if (!root_dead) {
        log->error("Failed to get point cloud tree from \"{}\"", inpath + "/dead");
        return false;
    }
    log->debug("Got pctree with {} children", root_dead->children().size());

    // BEE debug root_live
    if (!m_bee_dir.empty()) {
        std::string sub_dir = String::format("%s/%d", m_bee_dir, ident);
        Persist::assuredir(sub_dir);
        dump_bee(*root_live.get(), String::format("%s/%d-img.json", sub_dir, ident));
        dumpe_deadarea(*root_dead.get(), String::format("%s/%d-channel-deadarea.json", sub_dir, ident));
    }

    /// DEMO: iterate all clusters from root_live
    // std::unordered_map<std::string, std::chrono::milliseconds> timers;
    // for(const auto& cnode : root_live->children()) {
    //     // log->debug("cnode children: {}", cnode->children().size());
    //     Cluster pcc(cnode);
    //     start = std::chrono::high_resolution_clock::now();
    //     auto pos = pcc.calc_ave_pos(Point(0,0,0), 1e8, 0);
    //     end = std::chrono::high_resolution_clock::now();
    //     // log->debug("alg0 pos: {}", pos);
    //     duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //     timers["alg0"] += duration;
    //     start = std::chrono::high_resolution_clock::now();
    //     pos = pcc.calc_ave_pos(Point(0,0,0), 1e8, 1);
    //     end = std::chrono::high_resolution_clock::now();
    //     // log->debug("alg1 pos: {}", pos);
    //     duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    //     timers["alg1"] += duration;
    // }
    // log->debug("calc_ave_pos alg0 {} ms", timers["alg0"].count());
    // log->debug("calc_ave_pos alg1 {} ms", timers["alg1"].count());

    // dead_live
    // Calculate the length of all the clusters and save them into a map
    WireCell::PointCloud::Facade::TPCParams tp;
    std::map<const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster>, double> cluster_length_map;
    std::set<std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> > cluster_connected_dead;
    clustering_live_dead(root_live, root_dead, cluster_length_map, cluster_connected_dead, tp,
                         m_dead_live_overlap_offset);

    // BEE debug dead-live
    if (!m_bee_dir.empty()) {
        std::string sub_dir = String::format("%s/%d", m_bee_dir, ident);
        dump_bee(*root_live.get(), String::format("%s/%d-dead-live.json", sub_dir, ident));
    }

    std::string outpath = m_outpath;
    if (outpath.find("%") != std::string::npos) {
        outpath = String::format(outpath, ident);
    }
    start = std::chrono::high_resolution_clock::now();
    auto outtens = as_tensors(*root_live.get(), outpath + "/live");
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    log->debug("as_tensors live took {} ms", duration.count());

    start = std::chrono::high_resolution_clock::now();
    auto outtens_dead = as_tensors(*root_dead.get(), outpath + "/dead");
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    log->debug("as_tensors dead took {} ms", duration.count());

    // Merge
    /// TODO: is make_move_iterator faster?
    outtens.insert(outtens.end(), outtens_dead.begin(), outtens_dead.end());
    log->debug("Total outtens {} tensors", outtens.size());
    outts = as_tensorset(outtens, ints->ident());

    return true;
}
