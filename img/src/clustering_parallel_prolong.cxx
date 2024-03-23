#include <WireCellImg/ClusteringFuncs.h>

using namespace WireCell;
using namespace WireCell::Img;
using namespace WireCell::Aux;
using namespace WireCell::Aux::TensorDM;
using namespace WireCell::PointCloud::Facade;
using namespace WireCell::PointCloud::Tree;
void WireCell::PointCloud::Facade::clustering_parallel_prolong(
    Points::node_ptr& root_live,                                   // in/out
    Cluster::vector& live_clusters,
    std::map<const Cluster::pointer, double>& cluster_length_map,  // in/out
    std::set<Cluster::pointer>& cluster_connected_dead,            // in/out
    const TPCParams& tp,                                           // common params
    const double length_cut                                        //
)
{
  // prepare graph ...
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, int> Graph;
  Graph g;
  std::unordered_map<int, int> ilive2desc;  // added live index to graph descriptor
  std::map<const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster>, int> map_cluster_index;
  for (size_t ilive = 0; ilive < live_clusters.size(); ++ilive) {
    const auto& live = live_clusters[ilive];
    map_cluster_index[live] = ilive;
    ilive2desc[ilive] = boost::add_vertex(ilive, g);
  }

  // original algorithm ... (establish edges ... )
  Cluster::vector live_clusters_new;
  std::set<std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> > cluster_to_be_deleted;

  for (size_t i=0;i!=live_clusters.size();i++){
    auto cluster_1 = live_clusters.at(i);
    for (size_t j=i+1;j<live_clusters.size();j++){
      auto cluster_2 = live_clusters.at(j);
      if (Clustering_2nd_round(cluster_1,cluster_2,tp, cluster_length_map[cluster_1], cluster_length_map[cluster_2], length_cut)){
	//to_be_merged_pairs.insert(std::make_pair(cluster_1,cluster_2));
	boost::add_edge(ilive2desc[map_cluster_index[cluster_1]],
			ilive2desc[map_cluster_index[cluster_2]], g);
	cluster_to_be_deleted.insert(cluster_1);
	cluster_to_be_deleted.insert(cluster_2);
      }
    }
  }
  
    // form new clusters
  for (auto it = cluster_to_be_deleted.begin(); it != cluster_to_be_deleted.end(); it++) {
    cluster_length_map.erase(*it);
    cluster_connected_dead.erase(*it);
    // delete old clusters ...
    //	live_clusters.erase(find(live_clusters.begin(), live_clusters.end(), *it));
  }
  
  for (auto it = live_clusters.begin(); it != live_clusters.end(); it++) {
    if (cluster_to_be_deleted.find(*it) == cluster_to_be_deleted.end()) live_clusters_new.push_back(*it);
  }

  // Make new live node tree
  Points::node_ptr root_live_new = std::make_unique<Points::node_t>();
  //  debug("root_live {} root_live_new {}", root_live->children().size(), root_live_new->children().size());
  // std::unordered_set<Cluster::pointer> need_merging;
  // for (const auto& [dead, lives] : dead2lives) {
  //     if (lives.size() < 2) {
  //         continue;
  //     }
  //     debug("dead2lives size for dead cluster: {}", lives.size());
  //     need_merging.insert(lives.begin(), lives.end());
  //     auto cnode = root_live_new->insert(std::move(std::make_unique<Points::node_t>()));
  //     for (const auto& live : lives) {
  //         for (const auto& blob : live->m_blobs) {
  //             // this also removes blob node from root_live
  //             cnode->insert(blob->m_node);
  //         }
  //         // manually remove the cnode from root_live
  //         root_live->remove(live->m_node);
  //     }
  // }
  // debug("need_merging size: {}", need_merging.size());
  
  std::unordered_map<int, int> desc2id;
  std::unordered_map<int, std::set<int> > id2desc;
  int num_components = boost::connected_components(g, boost::make_assoc_property_map(desc2id));
  for (const auto& [desc, id] : desc2id) {
    id2desc[id].insert(desc);
  }
  //  debug("id2desc size: {}", id2desc.size());
  for (const auto& [id, descs] : id2desc) {
    if (descs.size() < 2) {
      continue;
    }
    
    //debug("id {} descs size: {}", id, descs.size());
    
    auto cnode1 = std::make_unique<Points::node_t>();
    for (const auto& desc : descs) {
      const int idx = g[desc];
      if (idx < 0) {  // no need anymore ...
	continue;
      }
      const auto& live = live_clusters[idx];
      for (const auto& blob : live->m_blobs) {
	// this also removes blob node from root_live
	cnode1->insert(blob->m_node);
      }
      // manually remove the cnode from root_live
      root_live->remove(live->m_node);
    }
    
    // new cluster information (need Haiwang to take a look at Facade ...)
    auto new_cluster = std::make_shared<Cluster>(cnode1);
    auto cnode = root_live_new->insert(std::move(cnode1));
    cluster_length_map[new_cluster] = new_cluster->get_length(tp);
    
    live_clusters_new.push_back(new_cluster);
    //	std::cout << "xin6:  " <<  cluster_length_map[new_cluster]/units::cm << std::endl;
    cluster_connected_dead.insert(new_cluster);
  }
  //  debug("root_live {} root_live_new {}", root_live->children().size(), root_live_new->children().size());
  
  // move remaining live clusters to new root
  for (auto& cnode : root_live->children()) {
    // this will NOT remove cnode from root_live, but set it to nullptr
    root_live_new->insert(std::move(cnode));
  }
  //  debug("root_live {} root_live_new {}", root_live->children().size(), root_live_new->children().size());
  // replace old with new
  root_live = std::move(root_live_new);
  
  live_clusters = std::move(live_clusters_new);
}


bool  WireCell::PointCloud::Facade::Clustering_2nd_round(const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster1,
							 const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster2,
							 const TPCParams& tp,                                           // common params
							 double length_1,
							 double length_2,
							 double length_cut){

  if (length_1 < 10*units::cm && length_2 < 10*units::cm) return false;
  

  std::shared_ptr<const WireCell::PointCloud::Facade::Blob> prev_mcell1 = 0;
  std::shared_ptr<const WireCell::PointCloud::Facade::Blob> prev_mcell2 = 0;
  std::shared_ptr<const WireCell::PointCloud::Facade::Blob> mcell1 = 0;
  geo_point_t p1;
  std::shared_ptr<const WireCell::PointCloud::Facade::Blob> mcell2 = 0;
  geo_point_t p2;

  double dis = WireCell::PointCloud::Facade::Find_Closest_Points(cluster1, cluster2, length_1, length_2, length_cut, mcell1, mcell2, p1,p2);

  if ((dis < length_cut || (dis < 80*units::cm && length_1 +length_2 > 50*units::cm && length_1>15*units::cm && length_2 > 15*units::cm))){
    geo_point_t cluster1_ave_pos = cluster1->calc_ave_pos(p1,10*units::cm);
    geo_point_t cluster2_ave_pos = cluster2->calc_ave_pos(p2,10*units::cm);

    bool flag_para = false;
    bool flag_para_U = false;
    bool flag_para_V = false;

    geo_point_t drift_dir(1, 0, 0);  // assuming the drift direction is along X ...
    double angle_u = tp.angle_u;
    double angle_v = tp.angle_v;
    double angle_w = tp.angle_w;
    
    // pronlonged case for U 3 and V 4 ...
    geo_point_t U_dir(0,cos(angle_u),sin(angle_u));
    geo_point_t V_dir(0,cos(angle_v),sin(angle_v));
    geo_point_t W_dir(0,cos(angle_w),sin(angle_w));

    
    // deal the parallel case ...
    if (length_1 > 10*units::cm && length_2 >10*units::cm){
      geo_point_t tempV1(p2.x() - p1.x(), p2.y() - p1.y(), p2.z() - p1.z());
      geo_point_t tempV2(cluster2_ave_pos.x() - cluster1_ave_pos.x(), cluster2_ave_pos.y() - cluster1_ave_pos.y(), cluster2_ave_pos.z() - cluster1_ave_pos.z());
      
      double angle1 = tempV1.angle(drift_dir);
      double angle4 = tempV2.angle(drift_dir);

      // looks like a parallel case
      if ( (fabs(angle1-3.1415926/2.)<10/180.*3.1415926 && dis > 10*units::cm ||
	    fabs(angle1-3.1415926/2.)<20/180.*3.1415926 && dis > 3*units::cm && dis <= 10*units::cm ||
	    fabs(angle1-3.1415926/2.)<45/180.*3.1415926 && dis <=3*units::cm)
	   && fabs(angle4-3.1415926/2.)<5/180.*3.1415926){
	
	geo_point_t dir1 = cluster1->vhough_transform(p1,60*units::cm,1); // cluster 1 direction based on hough
	geo_point_t dir2 = cluster2->vhough_transform(p2,60*units::cm,1); // cluster 2 direction based on hough
	
	double angle5 = dir1.angle(drift_dir);
	double angle6 = dir2.angle(drift_dir);
	
	if (fabs(angle5-3.1415926/2.)<5/180.*3.1415926 && fabs(angle6-3.1415926/2.)<20/180.*3.1415926 ||
	    fabs(angle5-3.1415926/2.)<20/180.*3.1415926 && fabs(angle6-3.1415926/2.)<5/180.*3.1415926){
	
	  flag_para = true;
	  
	  if (dis >= 3*length_1 && dis >= 3*length_2 && flag_para) return false;
	  
	  double angle2 = tempV1.angle(U_dir);
	  double angle3 = tempV1.angle(V_dir);

	  // look at parallel U
	  if ((fabs(angle2-3.1415926/2.)<7.5/180.*3.1415926 || (fabs(angle2-3.1415926/2.)<15/180.*3.1415926)&&dis <6*units::cm) && (dis<length_cut || (length_1 + length_2 > 100*units::cm)) && length_1 >15*units::cm && length_2 > 15*units::cm){
	    flag_para_U = true;

	    if ((length_1 < 25*units::cm || length_2 < 25*units::cm) && fabs(angle2-3.1415926/2.)<5.0/180.*3.1415926  && dis < 15* units::cm || dis < 3*units::cm){
	      // for short or small distance one
	      return true;
	    }else if (fabs(angle6-3.1415926/2.)/3.1415926*180.<1 && fabs(angle5-3.1415926/2.)/3.1415926*180.<1 && fabs(angle2-3.1415926/2.)<5.0/180.*3.1415926 && dis < 20*units::cm && (length_1 < 50*units::cm || length_2 < 50*units::cm)){
	      return true;
	    }else if (dis < 15*units::cm && (length_1 < 60*units::cm || length_2 < 60*units::cm) &&
		      fabs(angle2-3.1415926/2.)<2.5/180.*3.1415926){
	      // parallel case for reasonably short distance one
	      return true;
	    }else if (fabs(angle2-3.1415926/2.)<2.5/180.*3.1415926 && fabs(angle5-3.1415926/2.)<5/180.*3.1415926 && fabs(angle6-3.1415926/2.)<5/180.*3.141592 ){
	      // parallel case, but exclude both very long tracks
	      if (length_1 < 60*units::cm || length_2 < 60*units::cm){
		if (WireCell::PointCloud::Facade::is_angle_consistent(dir1,tempV1,false,15,angle_u,angle_v,angle_w) && WireCell::PointCloud::Facade::is_angle_consistent(dir2,tempV1,true,15,angle_u,angle_v,angle_w)) 
		  return true;
	      }else if (dis <5*units::cm){
		return true;
	      }else{
	      	double angle7 = (3.1415926-dir1.angle(dir2))/3.1415926*180.;
	      	double angle8 = (3.1415926-dir1.angle(tempV1))/3.1415926*180.; // dir1 = -p1, tempV1 = p2 - p1
	      	double angle9 = dir2.angle(tempV1)/3.1415926*180.; // dir2 = -p2
	      	if (angle7 < 30 && angle8 < 30 && angle9 < 30)
	      	  return true;
		if (WireCell::PointCloud::Facade::is_angle_consistent(dir1,tempV1,false,10,angle_u,angle_v,angle_w) && WireCell::PointCloud::Facade::is_angle_consistent(dir2,tempV1,true,10,angle_u,angle_v,angle_w)) 
		  return true; 
	      }
	      
	    }else{
	      // general case ... (not sure how useful though ...)
	      double angle7 = (3.1415926-dir1.angle(dir2))/3.1415926*180.;
	      double angle8 = (3.1415926-dir1.angle(tempV1))/3.1415926*180.; // dir1 = -p1, tempV1 = p2 - p1
	      double angle9 = dir2.angle(tempV1)/3.1415926*180.; // dir2 = -p2
	      if ((angle7 < 30 && angle8 < 30 && angle9 < 30 ||
		   fabs(angle5-3.1415926/2.)<5/180.*3.1415926 && fabs(angle6-3.1415926/2.)<5/180.*3.141592 &&
		   angle7 < 45 && angle8 < 45 && angle9 < 45) && dis < 20*units::cm)
		return true;
	      if (WireCell::PointCloud::Facade::is_angle_consistent(dir1,tempV1,false,10,angle_u,angle_v,angle_w) && WireCell::PointCloud::Facade::is_angle_consistent(dir2,tempV1,true,10,angle_u,angle_v,angle_w)) 
		return true; 
	    }
	  }

	  // look at parallel V
	  if ((fabs(angle3-3.1415926/2.)<7.5/180.*3.1415926 || (fabs(angle3-3.1415926/2.)<15/180.*3.1415926)&&dis <6*units::cm )&&(dis<length_cut || (length_1 + length_2 > 100*units::cm))&& length_1 >15*units::cm && length_2 > 15*units::cm){
	    flag_para_V = true;
	    //return true;
	    
	    if ((length_1 < 25*units::cm || length_2 < 25*units::cm) && fabs(angle3-3.1415926/2.)<5.0/180.*3.1415926 && dis < 15* units::cm || dis < 2*units::cm){
	      return true;
	    }else if (fabs(angle6-3.1415926/2.)/3.1415926*180.<1 && fabs(angle5-3.1415926/2.)/3.1415926*180.<1 && fabs(angle3-3.1415926/2.)<5.0/180.*3.1415926 && dis < 20*units::cm && (length_1 < 50*units::cm || length_2 < 50*units::cm)){
	      return true;
	    }else if (dis < 15*units::cm && fabs(angle3-3.1415926/2.)<2.5/180.*3.1415926 && (length_1 < 60*units::cm || length_2 < 60*units::cm) ){
	      return true;
	    }else if (fabs(angle3-3.1415926/2.)<2.5/180.*3.1415926 && fabs(angle5-3.1415926/2.)<5/180.*3.1415926 && fabs(angle6-3.1415926/2.)<5/180.*3.141592){
	      if (WireCell::PointCloud::Facade::is_angle_consistent(dir1,tempV1,false,15,angle_u,angle_v,angle_w) && WireCell::PointCloud::Facade::is_angle_consistent(dir2,tempV1,true,15,angle_u,angle_v,angle_w)) 
		return true;
	    }else{
	      double angle7 = (3.1415926-dir1.angle(dir2))/3.1415926*180.;
	      double angle8 = (3.1415926-dir1.angle(tempV1))/3.1415926*180.; // dir1 = -p1, tempV1 = p2 - p1
	      double angle9 = dir2.angle(tempV1)/3.1415926*180.; // dir2 = -p2
	      if (angle7 < 30 && angle8 < 30 && angle9 < 30||
		  fabs(angle5-3.1415926/2.)<5/180.*3.1415926 && fabs(angle6-3.1415926/2.)<5/180.*3.141592 &&
		  angle7 < 60 && angle8 < 60 && angle9 < 60)
		return true;
	      if (WireCell::PointCloud::Facade::is_angle_consistent(dir1,tempV1,false,10,angle_u,angle_v,angle_w) && WireCell::PointCloud::Facade::is_angle_consistent(dir2,tempV1,true,10,angle_u,angle_v,angle_w))
	      	  return true;
	    }
	  }
	}
      }
    }

     // look at prolonged case ... (add W case) 
    {
      geo_point_t tempV1(0, p2.y() - p1.y(), p2.z() - p1.z());
      geo_point_t tempV5;
      double angle1 = tempV1.angle(U_dir);
      tempV5.set(fabs(p2.x()-p1.x()),sqrt(pow(p2.y() - p1.y(),2)+pow(p2.z() - p1.z(),2))*sin(angle1),0);
      angle1 = tempV5.angle(drift_dir);
      
      double angle2 = tempV1.angle(V_dir);
      tempV5.set(fabs(p2.x()-p1.x()),sqrt(pow(p2.y() - p1.y(),2)+pow(p2.z() - p1.z(),2))*sin(angle2),0);
      angle2 = tempV5.angle(drift_dir);

      double angle1p = tempV1.angle(W_dir);
      tempV5.set(fabs(p2.x()-p1.x()),sqrt(pow(p2.y() - p1.y(),2)+pow(p2.z() - p1.z(),2))*sin(angle1p),0);
      angle1p = tempV5.angle(drift_dir);

      if (angle1<7.5/180.*3.1415926  ||
	  angle2<7.5/180.*3.1415926  ||
	  angle1p<7.5/180.*3.1415926 ){
	if (length_1 > 10*units::cm || length_2 > 10*units::cm){
	  geo_point_t dir1 = cluster1->vhough_transform(p1,60*units::cm,1); // cluster 1 direction based on hough
	  geo_point_t dir2 = cluster2->vhough_transform(p2,60*units::cm,1); // cluster 1 direction based on hough
	  geo_point_t dir3(p2.x()-p1.x(),p2.y()-p1.y(),p2.z()-p1.z());
	  double angle3 = dir3.angle(dir2);
	  double angle4 = 3.1415926-dir3.angle(dir1);

	  if ((angle3<25/180.*3.1415926 || length_2<10*units::cm)&&(angle4<25/180.*3.1415926|| length_1<10*units::cm)&&dis<5*units::cm ||
	      (angle3<15/180.*3.1415926 || length_2<10*units::cm)&&(angle4<15/180.*3.1415926|| length_1<10*units::cm)&&dis<15*units::cm ||
	      (angle3<7.5/180.*3.1415926 || length_2<10*units::cm)&&(angle4<7.5/180.*3.1415926|| length_1<10*units::cm) ||
	      (angle3+angle4 < 15/180.*3.1415926 && angle3 < 10/180.*3.1415926 && angle4 < 10/180.*3.1415926)
	      )
	    return true;
	}
      }else{
      	//regular cases (only for very short distance ... )
      	if (dis < 5*units::cm){
      	  if (length_1 > 10*units::cm && length_2 >10*units::cm){
      	    geo_point_t dir1 = cluster1->vhough_transform(p1,30*units::cm,1); // cluster 1 direction based on hough
      	    geo_point_t dir2 = cluster2->vhough_transform(p2,30*units::cm,1); // cluster 1 direction based on hough
      	    geo_point_t dir3(p2.x()-p1.x(),p2.y()-p1.y(),p2.z()-p1.z());
      	    double angle3 = dir3.angle(dir2);
      	    double angle4 = 3.1415926-dir3.angle(dir1);

      	    //std::cout << angle3/3.1415926*180. << " " << angle4/3.1415926*180. << std::endl;
      	    if ((angle3<15/180.*3.1415926 || length_2<6*units::cm)
      		&& (angle4<15/180.*3.1415926|| length_1<6*units::cm))
      	      return true;
      	  }
      	}
      }
    }
  }
  return false;
}
