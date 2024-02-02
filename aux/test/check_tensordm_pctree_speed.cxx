#include "WireCellAux/TensorDMpointtree.h"
#include "WireCellUtil/PointTree.h"
#include "WireCellUtil/NamedFactory.h"
#include "WireCellIface/IConfigurable.h"
#include "WireCellIface/ITensorSetSource.h"

#include "WireCellUtil/Logging.h"
using spdlog::debug;
using spdlog::info;
using spdlog::error;

using namespace WireCell;
using namespace WireCell::PointCloud;
using namespace WireCell::Aux::TensorDM;
using namespace WireCell::PointCloud::Tree;

int main(int argc, char* argv[])
{
    Log::default_logging();

    if (argc < 4) {
        error("usage: check_tensordm_pctree_speed filename prefix datapath");
        return 1;
    }

    auto cobj = Factory::find_tn<IConfigurable>("TensorFileSource");
    auto cfg = cobj->default_configuration();
    cfg["inname"] = argv[1];
    cfg["prefix"] = argv[2];
    std::string datapath = argv[3];
    cobj->configure(cfg);
    auto in = Factory::find_tn<ITensorSetSource>("TensorFileSource");

    ITensorSet::pointer ts;
    (*in)(ts);


    // auto pct = as_pctree(*tens, datapath);
    // auto root = make_simple_pctree();
    // const std::string datapath = "root";
    // auto tens = as_tensors(*root.get(), datapath);
    // CHECK(tens.size() > 0);

    // debug("{:20} {}", "datatype", "datapath");
    // for (auto ten : tens) {
    //     auto md = ten->metadata();
    //     debug("{:20} {}", md["datatype"].asString(), md["datapath"].asString());
    // }

    auto tensp = ts->tensors();
    auto start = std::chrono::high_resolution_clock::now();
    auto root = as_pctree(*tensp, datapath);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    info("as_pctree for {} took {} ms", datapath, duration.count());

    return 0;
}
