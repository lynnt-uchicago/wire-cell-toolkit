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
using WireCell::PointCloud::Cluster;

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
    log->debug("After merging, Got {} tensors", intens.size());
    const auto& root = as_pctree(intens, inpath);
    if (!root) {
        log->error("Failed to get point cloud tree from \"{}\"", inpath);
        return false;
    }
    log->debug("Got pctree with {} children", root->children().size());

    /// DEMO: iterate all clusters from root
    for(const auto& cnode : root->children()) {
        log->debug("cnode children: {}", cnode->children().size());
        Cluster pcc(cnode);
        auto pos = pcc.calc_ave_pos(Point(0,0,0), 1e8);
        log->debug("pos: {}", pos);
    }

    std::string outpath = m_outpath;
    if (outpath.find("%") != std::string::npos) {
        outpath = String::format(outpath, ident);
    }
    auto outtens = as_tensors(*root.get(), outpath);
    log->debug("Made {} tensors", outtens.size());
    outts = as_tensorset(outtens, ints->ident());

    return true;
}
