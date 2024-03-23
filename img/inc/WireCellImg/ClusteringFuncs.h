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
    // first function ...
    void clustering_live_dead(Points::node_ptr& root_live,                                   // in/out
			      //                             const Points::node_ptr& root_dead,                             // in
			      Cluster::vector& live_clusters,
			      const Cluster::vector& dead_clusters,
                              std::map<const Cluster::pointer, double>& cluster_length_map,  // in/out
                              std::set<Cluster::pointer>& cluster_connected_dead,            // in/out
                              const TPCParams& tp,                                           // common params
                              const int dead_live_overlap_offset                             // specific params
    );

    //helper function ..
    double Find_Closest_Points(const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster1,
			       const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster2,
			       double length_1,
			       double length_2,
			       double length_cut,
			       std::shared_ptr<const WireCell::PointCloud::Facade::Blob> mcell1,
			       std::shared_ptr<const WireCell::PointCloud::Facade::Blob> mcell2,
			       geo_point_t& p1_save,
			       geo_point_t& p2_save
			       );
			       
    
    // second function ...
    void clustering_extend(Points::node_ptr& root_live,                                   // in/out
			   Cluster::vector& live_clusters,
                           std::map<const Cluster::pointer, double>& cluster_length_map,  // in/out
                           std::set<Cluster::pointer>& cluster_connected_dead,            // in/out
                           const TPCParams& tp,                                           // common params
                           const int flag,                                                //
                           const double length_cut,                                       //
                           const int num_try,                                             //
                           const double length_2_cut,                                     //
                           const int num_dead_try                                         //
			   );
    bool Clustering_4th_prol(const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster1,
			     const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster2,
			     const TPCParams& tp,                                           // common params
			     double length_2,
			     geo_point_t& earliest_p,
			     geo_point_t& dir_earlp,
			     double length_cut);
    
    bool Clustering_4th_para(const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster1,
			     const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster2,
			     const TPCParams& tp,                                           // common params
			     double length_1, double length_2,
			     geo_point_t& earliest_p,
			     geo_point_t& dir_earlp,
			     double length_cut);
    bool Clustering_4th_reg(const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster1,
			    const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster2,
			    const TPCParams& tp,                                           // common params
			    double length_1, double length_2,
			    geo_point_t p1, double length_cut);
    bool Clustering_4th_dead(const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster1,
			     const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster2,
			     const TPCParams& tp,                                           // common params
			     double length_1, double length_2, double length_cut, int num_dead_try=3);
      

    // third function 
    void clustering_regular(Points::node_ptr& root_live,                                   // in/out
			    Cluster::vector& live_clusters,
                            std::map<const Cluster::pointer, double>& cluster_length_map,  // in/out
                            std::set<Cluster::pointer>& cluster_connected_dead,            // in/out
                            const TPCParams& tp,                                           // common params
                            const double length_cut,                                       //
                            bool flag_enable_extend                                        //
    );
    bool Clustering_1st_round(const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster1,
			      const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster2,
			      const TPCParams& tp,                                           // common params
			      double length_1,
			      double length_2,
			      double length_cut = 45*units::cm,
			      bool flag_enable_extend = true);

    //
    void clustering_parallel_prolong(Points::node_ptr& root_live,                                   // in/out
				     Cluster::vector& live_clusters,
                                     std::map<const Cluster::pointer, double>& cluster_length_map,  // in/out
                                     std::set<Cluster::pointer>& cluster_connected_dead,            // in/out
                                     const TPCParams& tp,                                           // common params
                                     const double length_cut                                        //
    );
    bool Clustering_2nd_round(const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster1,
			      const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster2,
			      double length_1,
			      double length_2,
			      double length_cut = 35*units::cm);
    
    
    //
    void clustering_close(Points::node_ptr& root_live,                                   // in/out
			  Cluster::vector& live_clusters,
                          std::map<const Cluster::pointer, double>& cluster_length_map,  // in/out
                          std::set<Cluster::pointer>& cluster_connected_dead,            // in/out
                          const TPCParams& tp,                                           // common params
                          const double length_cut                                        //
    );
    bool Clustering_3rd_round( const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster1,
			       const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster2,
			       double length_1,
			       double length_2,
			       double length_cut = 1*units::cm);

    
}  // namespace WireCell::PointCloud::Facade
