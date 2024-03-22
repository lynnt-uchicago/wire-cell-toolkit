#include <WireCellImg/ClusteringFuncs.h>

using namespace WireCell;
using namespace WireCell::Img;
using namespace WireCell::Aux;
using namespace WireCell::Aux::TensorDM;
using namespace WireCell::PointCloud::Facade;
using namespace WireCell::PointCloud::Tree;
void WireCell::PointCloud::Facade::clustering_extend(
    Points::node_ptr& root_live,                                   // in/out
    std::map<const Cluster::pointer, double>& cluster_length_map,  // in/out
    std::set<Cluster::pointer>& cluster_connected_dead,            // in/out
    const TPCParams& tp,                                           // common params
    const int flag,                                                //
    const double length_cut,                                       //
    const int num_try,                                             //
    const double length_2_cut,                                     //
    const int num_dead_try                                         //
)
{
  
  
}
