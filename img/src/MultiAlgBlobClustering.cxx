#include "WireCellImg/MultiAlgBlobClustering.h"
#include "WireCellImg/PointCloudFacade.h"
#include "WireCellUtil/NamedFactory.h"
#include "WireCellUtil/Units.h"
#include "WireCellAux/TensorDMpointtree.h"
#include "WireCellAux/TensorDMdataset.h"
#include "WireCellAux/TensorDMcommon.h"
#include "WireCellAux/SimpleTensorSet.h"

WIRECELL_FACTORY(MultiAlgBlobClustering, WireCell::Img::MultiAlgBlobClustering,
                 WireCell::INamed,
                 WireCell::ITensorSetFilter,
                 WireCell::IConfigurable)

using namespace WireCell;
using namespace WireCell::Img;
using namespace WireCell::Aux;
using namespace WireCell::Aux::TensorDM;
using namespace WireCell::PointCloud::Facade;

MultiAlgBlobClustering::MultiAlgBlobClustering()
    : Aux::Logger("MultiAlgBlobClustering", "img")
{
}

void MultiAlgBlobClustering::configure(const WireCell::Configuration& cfg)
{
    m_inpath = get(cfg, "inpath", m_inpath);
    m_outpath = get(cfg, "outpath", m_outpath);
}

WireCell::Configuration MultiAlgBlobClustering::default_configuration() const
{
    Configuration cfg;
    cfg["inpath"] = m_inpath;
    cfg["outpath"] = m_outpath;
    return cfg;
}


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
    const auto& root_live = as_pctree(intens, inpath+"/live");
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    log->debug("as_pctree for {} took {} ms", inpath+"/live", duration.count());
    if (!root_live) {
        log->error("Failed to get point cloud tree from \"{}\"", inpath);
        return false;
    }
    log->debug("Got pctree with {} children", root_live->children().size());

    start = std::chrono::high_resolution_clock::now();
    const auto& root_dead = as_pctree(intens, inpath+"/dead");
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    log->debug("as_pctree for {} took {} ms", inpath+"/dead", duration.count());
    if (!root_dead) {
        log->error("Failed to get point cloud tree from \"{}\"", inpath+"/dead");
        return false;
    }
    log->debug("Got pctree with {} children", root_dead->children().size());

    /// DEMO: iterate all clusters from root_live
    std::unordered_map<std::string, std::chrono::milliseconds> timers;
    for(const auto& cnode : root_live->children()) {
        // log->debug("cnode children: {}", cnode->children().size());
        Cluster pcc(cnode);
        start = std::chrono::high_resolution_clock::now();
        auto pos = pcc.calc_ave_pos(Point(0,0,0), 1e8, 0);
        end = std::chrono::high_resolution_clock::now();
        // log->debug("alg0 pos: {}", pos);
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        timers["alg0"] += duration;
        start = std::chrono::high_resolution_clock::now();
        pos = pcc.calc_ave_pos(Point(0,0,0), 1e8, 1);
        end = std::chrono::high_resolution_clock::now();
        // log->debug("alg1 pos: {}", pos);
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        timers["alg1"] += duration;
    }
    log->debug("calc_ave_pos alg0 {} ms", timers["alg0"].count());
    log->debug("calc_ave_pos alg1 {} ms", timers["alg1"].count());

    std::string outpath = m_outpath;
    if (outpath.find("%") != std::string::npos) {
        outpath = String::format(outpath, ident);
    }
    start = std::chrono::high_resolution_clock::now();
    auto outtens = as_tensors(*root_live.get(), outpath+"/live");
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    log->debug("as_tensors live took {} ms", duration.count());

    start = std::chrono::high_resolution_clock::now();
    auto outtens_dead = as_tensors(*root_dead.get(), outpath+"/dead");
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
