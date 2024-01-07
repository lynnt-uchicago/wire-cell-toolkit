#include "WireCellImg/PointCloudFacade.h"

using namespace WireCell;
using namespace WireCell::PointCloud;
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

WireCell::PointCloud::Point Cluster::calc_ave_pos(const Point& origin, const double dis) const {
    spdlog::set_level(spdlog::level::debug); // Set global log level to debug
    /// FIXME: there are many assumptions made, shoud we check these assumptions?
    /// a bit worriying about the speed.
    Scope scope = { "3d", {"x","y","z"} };
    const auto& sv = m_node->value.scoped_view(scope);
    // const auto& spcs = sv.pcs();
    debug("sv {}", dump_pcs(sv.pcs()));
    const auto& skd = sv.kd();
    auto rad = skd.radius(dis, origin);
    /// FIXME: what if rad is empty?
    if(rad.size() == 0) {
        raise<ValueError>("empty point cloud");
    }
    const auto& snodes = sv.nodes();
    std::set<size_t> maj_inds;
    for (size_t pt_ind = 0; pt_ind<rad.size(); ++pt_ind) {
        auto& [pit,dist] = rad[pt_ind];
        const auto [maj_ind,min_ind] = pit.index();
        maj_inds.insert(maj_ind);
    }
    debug("maj_inds.size() {} ", maj_inds.size());
    Point ret(0,0,0);
    double total_charge{0};
    for (size_t maj_ind : maj_inds) {
        const auto* node = snodes[maj_ind];
        const auto& lpcs = node->value.local_pcs();
        const auto& pc_scalar = lpcs.at("scalar");
        const auto charge = pc_scalar.get("charge")->elements<double>()[0];
        const auto center_x = pc_scalar.get("center_x")->elements<double>()[0];
        const auto center_y = pc_scalar.get("center_y")->elements<double>()[0];
        const auto center_z = pc_scalar.get("center_z")->elements<double>()[0];
        debug("charge {} center {{{} {} {}}}", charge, center_x, center_y, center_z);
        Point inc(center_x, center_y, center_z);
        inc = inc * charge;
        ret += inc;
        total_charge += charge;
    }
    if (total_charge != 0) {
        ret = ret / total_charge;
    }
    debug("ret {{{} {} {}}}", ret.x(), ret.y(), ret.z());
    return ret;
}

double Blob::center_pos() const { return 0; }