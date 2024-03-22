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

namespace WireCell::PointCloud::Facade {
    using namespace WireCell::PointCloud::Tree;
    void clustering_live_dead(Points::node_ptr& root_live,                                   // in/out
			      //                             const Points::node_ptr& root_dead,                             // in
			      Cluster::vector& live_clusters,
			      const Cluster::vector& dead_clusters,
                              std::map<const Cluster::pointer, double>& cluster_length_map,  // in/out
                              std::set<Cluster::pointer>& cluster_connected_dead,            // in/out
                              const TPCParams& tp,                                           // common params
                              const int dead_live_overlap_offset                             // specific params
    );
    void clustering_extend(Points::node_ptr& root_live,                                   // in/out
                           std::map<const Cluster::pointer, double>& cluster_length_map,  // in/out
                           std::set<Cluster::pointer>& cluster_connected_dead,            // in/out
                           const TPCParams& tp,                                           // common params
                           const int flag,                                                //
                           const double length_cut,                                       //
                           const int num_try,                                             //
                           const double length_2_cut,                                     //
                           const int num_dead_try                                         //
    );

    void clustering_regular(Points::node_ptr& root_live,                                   // in/out
                            std::map<const Cluster::pointer, double>& cluster_length_map,  // in/out
                            std::set<Cluster::pointer>& cluster_connected_dead,            // in/out
                            const TPCParams& tp,                                           // common params
                            const double length_cut,                                       //
                            bool flag_enable_extend                                        //
    );

    void clustering_parallel_prolong(Points::node_ptr& root_live,                                   // in/out
                                     std::map<const Cluster::pointer, double>& cluster_length_map,  // in/out
                                     std::set<Cluster::pointer>& cluster_connected_dead,            // in/out
                                     const TPCParams& tp,                                           // common params
                                     const double length_cut                                        //
    );

    void clustering_close(Points::node_ptr& root_live,                                   // in/out
                          std::map<const Cluster::pointer, double>& cluster_length_map,  // in/out
                          std::set<Cluster::pointer>& cluster_connected_dead,            // in/out
                          const TPCParams& tp,                                           // common params
                          const double length_cut                                        //
    );
}  // namespace WireCell::PointCloud::Facade
