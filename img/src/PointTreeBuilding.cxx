#include "WireCellImg/PointTreeBuilding.h"
#include "WireCellUtil/PointTree.h"
#include "WireCellUtil/GraphTools.h"
#include "WireCellUtil/NamedFactory.h"

#include "WireCellAux/ClusterHelpers.h"
#include "WireCellAux/TensorDMpointtree.h"
#include "WireCellAux/TensorDMcommon.h"

WIRECELL_FACTORY(PointTreeBuilding, WireCell::Img::PointTreeBuilding,
                 WireCell::INamed,
                 WireCell::IClusterTensorSet,
                 WireCell::IConfigurable)

using namespace WireCell;
using namespace WireCell::GraphTools;
using namespace WireCell::Img;
using namespace WireCell::Aux;
using namespace WireCell::Aux::TensorDM;
using namespace WireCell::PointCloud::Tree;

PointTreeBuilding::PointTreeBuilding()
    : Aux::Logger("PointTreeBuilding", "img")
{
}


PointTreeBuilding::~PointTreeBuilding()
{
}


void PointTreeBuilding::configure(const WireCell::Configuration& cfg)
{
    m_datapath = get(cfg, "datapath", m_datapath);
    auto samplers = cfg["samplers"];
    if (samplers.isNull()) {
        raise<ValueError>("add at least one entry to the \"samplers\" configuration parameter");
    }

    for (auto name : samplers.getMemberNames()) {
        auto tn = samplers[name].asString();
        if (tn.empty()) {
            raise<ValueError>("empty type/name for sampler \"%s\"", name);
        }
        log->debug("point cloud \"{}\" will be made by sampler \"{}\"",
                   name, tn);
        m_samplers[name] = Factory::find_tn<IBlobSampler>(tn); 
    }

}


WireCell::Configuration PointTreeBuilding::default_configuration() const
{
    Configuration cfg;
    // eg:
    //    cfg["samplers"]["samples"] = "BlobSampler";
    cfg["datapath"] = m_datapath;
    return cfg;
}

bool PointTreeBuilding::operator()(const input_pointer& icluster, output_pointer& tensorset)
{
    tensorset = nullptr;

    if (!icluster) {
        log->debug("EOS at call {}", m_count++);
        return true;
    }

    const auto& gr = icluster->graph();
    log->debug("load cluster {} at call={}: {}", icluster->ident(), m_count, dumps(gr));

    size_t nblobs = 0;
    Points::node_ptr root = std::make_unique<Points::node_t>();
    for (const auto& vdesc : mir(boost::vertices(gr))) {
        const char code = gr[vdesc].code();
        if (code != 'b') {
            continue;
        }
        auto iblob = std::get<IBlob::pointer>(gr[vdesc].ptr);
        named_pointclouds_t pcs;
        for (auto& [name, sampler] : m_samplers) {
            pcs.emplace(name, sampler->sample_blob(iblob, nblobs));
        }
        root->insert(Points(pcs));
        ++nblobs;
    }

    const int ident = icluster->ident();
    std::string datapath = m_datapath;
    if (datapath.find("%") != std::string::npos) {
        datapath = String::format(datapath, ident);
    }
    auto tens = as_tensors(*root.get(), datapath);
    tensorset = as_tensorset(tens, ident);

    log->debug("sampled {} blobs from set {} making {} tensors at call {}",
               nblobs, ident, tens.size(), m_count++);

    return true;
}

        

