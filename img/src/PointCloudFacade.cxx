#include "WireCellImg/PointCloudFacade.h"

using namespace WireCell;
using namespace WireCell::PointCloud;
using namespace WireCell::PointCloud::Tree; // for "Points" node value type

#include "WireCellUtil/Logging.h"
using spdlog::debug;

WireCell::PointCloud::Point Cluster::calc_ave_pos(const Point& origin, const double dis) const {
    spdlog::set_level(spdlog::level::debug); // Set global log level to debug
    /// FIXME: there are many assumptions made, shoud we check these assumptions?
    /// a bit worriying about the speed.
    Scope scope = { "3d", {"x","y","z"} };
    const auto& sv = m_node->value.scoped_view(scope);
    const auto& skd = sv.kd();
    auto rad = skd.radius(dis, origin);
    const auto& snodes = sv.nodes();
    std::set<size_t> maj_inds;
    for (size_t pt_ind = 0; pt_ind<rad.size(); ++pt_ind) {
        auto& [pit,dist] = rad[pt_ind];
        const size_t maj_ind = skd.major_index(pit);
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