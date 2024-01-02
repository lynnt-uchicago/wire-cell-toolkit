#include "WireCellImg/PointTreeBuilding.h"
#include "WireCellImg/Projection2D.h"
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
using WireCell::Img::Projection2D::get_geom_clusters;
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

namespace {

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
    // dump a NaryTree node
    std::string dump_node(const WireCell::PointCloud::Tree::Points::node_t* node)
    {
        std::stringstream ss;
        ss << "node: " << node;
        if (node) {
            const auto& lpcs = node->value.local_pcs();
            ss << " with " << lpcs.size() << " local pcs";
            for (const auto& [name, pc] : lpcs) {
                ss << " " << name << ": " << dump_ds(pc);
            }
        } else {
            ss << " null";
        }
        return ss.str();
    }
    // dump childs of a NaryTree node
    std::string dump_children(const WireCell::PointCloud::Tree::Points::node_t* root)
    {
        std::stringstream ss;
        ss << "NaryTree: " << root->children().size() << " children";
        const Points::node_ptr& first = root->children().front();
        ss << dump_node(first.get());
        return ss.str();
    }
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

    auto clusters = get_geom_clusters(gr);
    log->debug("got {} clusters", clusters.size());
    size_t nblobs = 0;
    Points::node_ptr root = std::make_unique<Points::node_t>();
    for (auto& [cluster_id, vdescs] : clusters) {
        auto cnode = root->insert(std::move(std::make_unique<Points::node_t>()));
        for (const auto& vdesc : vdescs) {
            const char code = gr[vdesc].code();
            if (code != 'b') {
                continue;
            }
            auto iblob = std::get<IBlob::pointer>(gr[vdesc].ptr);
            named_pointclouds_t pcs;
            for (auto& [name, sampler] : m_samplers) {
                /// TODO: use nblobs or iblob->ident()?
                pcs.emplace(name, sampler->sample_blob(iblob, nblobs));
            }
            cnode->insert(Points(pcs));
            // log->debug("pcs {} cnode {}", pcs.size(), dump_children(cnode));
            // for (const auto& [name, pc] : pcs) {
            //     log->debug("{} -> keys {} size_major {}", name, pc.keys().size(), pc.size_major());
            // }
            ++nblobs;
        }
        /// DEBUGONLY
        // if (nblobs > 1000) {
        //     break;
        // }
    }

    const int ident = icluster->ident();
    std::string datapath = m_datapath;
    if (datapath.find("%") != std::string::npos) {
        datapath = String::format(datapath, ident);
    }
    auto tens = as_tensors(*root.get(), datapath);
    log->debug("Made {} tensors", tens.size());
    tensorset = as_tensorset(tens, ident);

    log->debug("sampled {} blobs from set {} making {} tensors at call {}",
               nblobs, ident, tens.size(), m_count++);

    return true;
}

        

