#include <WireCellImg/ClusteringFuncs.h>

using namespace WireCell;
using namespace WireCell::Img;
using namespace WireCell::Aux;
using namespace WireCell::Aux::TensorDM;
using namespace WireCell::PointCloud::Facade;
using namespace WireCell::PointCloud::Tree;
void WireCell::PointCloud::Facade::clustering_extend(
    Points::node_ptr& root_live,                                   // in/out
    Cluster::vector& live_clusters,
    std::map<const Cluster::pointer, double>& cluster_length_map,  // in/out
    std::set<Cluster::pointer>& cluster_connected_dead,            // in/out
    const TPCParams& tp,                                           // common params
    const int flag,                                                //
    const double length_cut,                                       //
    const int num_try,                                             //
    const double length_2_cut,                                     //
    const int num_dead_try                                         //
){
  
  
}





bool WireCell::PointCloud::Facade::Clustering_4th_prol(const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster1,
						       const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster2,
						       const TPCParams& tp,                                           // common params
						       double length_2,
						       geo_point_t& earliest_p,
						       geo_point_t& dir_earlp,
						       double length_cut){
  
}

bool WireCell::PointCloud::Facade::Clustering_4th_para(const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster1,
						       const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster2,
						       const TPCParams& tp,                                           // common params
						       double length_1, double length_2,
						       geo_point_t& earliest_p,
						       geo_point_t& dir_earlp,
						       double length_cut){
  
}

bool WireCell::PointCloud::Facade::Clustering_4th_reg(const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster1,
						      const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster2,
						      const TPCParams& tp,                                           // common params
						      double length_1, double length_2,
						      geo_point_t p1, double length_cut){

}

double WireCell::PointCloud::Facade::Find_Closest_Points(const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster1,
			       const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster2,
			       double length_1,
			       double length_2,
			       double length_cut,
			       std::shared_ptr<const WireCell::PointCloud::Facade::Blob> mcell1_save,
			       std::shared_ptr<const WireCell::PointCloud::Facade::Blob> mcell2_save,
			       geo_point_t& p1_save,
			       geo_point_t& p2_save
			   ){
  double dis_save = 1e9;
  
  std::shared_ptr<const WireCell::PointCloud::Facade::Blob> prev_mcell1 = 0;
  std::shared_ptr<const WireCell::PointCloud::Facade::Blob> prev_mcell2 = 0;
  std::shared_ptr<const WireCell::PointCloud::Facade::Blob> mcell1 = 0;
  geo_point_t p1;
  std::shared_ptr<const WireCell::PointCloud::Facade::Blob> mcell2 = 0;
  geo_point_t p2;
  double dis;

  if (length_1 < length_2){
    mcell1 = cluster1->get_first_blob();
    p1 = mcell1->center_pos();

    while (mcell1 != prev_mcell1 || mcell2 != prev_mcell2){
      prev_mcell1 = mcell1;
      prev_mcell2 = mcell2;

      auto temp_results = cluster2->get_closest_point_mcell(p1);
      p2 = temp_results.first;
      mcell2 = temp_results.second;

      temp_results = cluster1->get_closest_point_mcell(p2);
      p1 = temp_results.first;
      mcell1 = temp_results.second;
    }
    geo_point_t diff = p1 - p2;
    dis = diff.magnitude();

    if (dis < dis_save){
      dis_save = dis;
      mcell1_save = mcell1;
      mcell2_save = mcell2;
      p1_save = p1;
      p2_save = p2;
    }

    prev_mcell1 = 0;
    prev_mcell2 = 0;

    mcell1 = cluster1->get_last_blob();
    p1 = mcell1->center_pos();

    while(mcell1!=prev_mcell1 || mcell2!=prev_mcell2){
      prev_mcell1 = mcell1;
      prev_mcell2 = mcell2;
      
      // find the closest point and merged cell in cluster2
      auto temp_results = cluster2->get_closest_point_mcell(p1);
      p2 = temp_results.first;
      mcell2 = temp_results.second;
      // find the closest point and merged cell in cluster1
      temp_results = cluster1->get_closest_point_mcell(p2);
      p1 = temp_results.first;
      mcell1 = temp_results.second;
    }
    diff = p1 - p2;
    dis = diff.magnitude();

    if (dis < dis_save){
      dis_save = dis;
      mcell1_save = mcell1;
      mcell2_save = mcell2;
      p1_save = p1;
      p2_save = p2;
    }
    
    
  }else{


    mcell2 = cluster2->get_first_blob();
    p2 = mcell2->center_pos();

    while(mcell1!=prev_mcell1 || mcell2!=prev_mcell2){
      prev_mcell1 = mcell1;
      prev_mcell2 = mcell2;
      
      // find the closest point and merged cell in cluster2
      auto temp_results = cluster1->get_closest_point_mcell(p2);
      p1 = temp_results.first;
      mcell1 = temp_results.second;
      
      // find the closest point and merged cell in cluster1
      temp_results = cluster2->get_closest_point_mcell(p1);
      p2 = temp_results.first;
      mcell2 = temp_results.second;
    }
    geo_point_t diff = p1 - p2;
    dis = diff.magnitude();

    if (dis < dis_save){
      dis_save = dis;
      mcell1_save = mcell1;
      mcell2_save = mcell2;
      p1_save = p1;
      p2_save = p2;
    }

    
    prev_mcell1 = 0;
    prev_mcell2 = 0;
    

    mcell2 = cluster2->get_last_blob();
    p2 = mcell2->center_pos();

    while(mcell1!=prev_mcell1 || mcell2!=prev_mcell2){
      prev_mcell1 = mcell1;
      prev_mcell2 = mcell2;
      
      // find the closest point and merged cell in cluster2
      auto temp_results = cluster1->get_closest_point_mcell(p2);
      p1 = temp_results.first;
      mcell1 = temp_results.second;
      // find the closest point and merged cell in cluster1
      temp_results = cluster2->get_closest_point_mcell(p1);
      p2 = temp_results.first;
      mcell2 = temp_results.second;
    }
    diff = p1 - p2;
    dis = diff.magnitude();

    if (dis < dis_save){
      dis_save = dis;
      mcell1_save = mcell1;
      mcell2_save = mcell2;
      p1_save = p1;
      p2_save = p2;
    }


    
  }

  return dis_save;
  
}


bool WireCell::PointCloud::Facade::Clustering_4th_dead(const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster_1,
						       const std::shared_ptr<const WireCell::PointCloud::Facade::Cluster> cluster_2,
						       const TPCParams& tp,                                           // common params
						       double length_1, double length_2, double length_cut, int num_dead_try){
  
  geo_point_t drift_dir(1, 0, 0);  // assuming the drift direction is along X ...
  double angle_u = tp.angle_u;
  double angle_v = tp.angle_v;
  double angle_w = tp.angle_w;

  std::shared_ptr<const WireCell::PointCloud::Facade::Blob> mcell1 = 0;
  std::shared_ptr<const WireCell::PointCloud::Facade::Blob> mcell2 = 0;
  geo_point_t p1;
  geo_point_t p2;
  
  double dis = Find_Closest_Points(cluster_1, cluster_2, length_1, length_2, length_cut, mcell1, mcell2, p1,p2);

  //add a special one ...  for uboone ...
  if (length_1 > 30*units::cm && length_2 > 30*units::cm &&
      (dis < 3*units::cm ||
       fabs(p1.x()-p2.x()) < 1.6*units::cm && (dis < 20*units::cm &&
						     p1.z() > 700.6*units::cm && p1.z() < 739.6*units::cm &&     // special region
						     p2.z() > 700.6*units::cm && p2.z() < 739.6*units::cm &&     // special region ...
						     p1.y() > -10.4*units::cm && p1.y() < 29*units::cm &&
						     p2.y() > -10.4*units::cm && p2.y() < 29*units::cm )
       )){
    return true; 
  }

  if ((dis < length_cut || (length_2 > 50*units::cm && dis < 80*units::cm))){

    geo_point_t cluster1_ave_pos_save;
    geo_point_t cluster2_ave_pos_save;
    geo_point_t dir1_save;
    geo_point_t dir3_save;

    for (int i=0;i!=num_dead_try;i++){
      geo_point_t cluster1_ave_pos; 
      geo_point_t cluster2_ave_pos; 
      
      geo_point_t dir1; 
      geo_point_t dir3; 
      geo_point_t dir2;

      if (i==0){
	cluster1_ave_pos = cluster_1->calc_ave_pos(p1,5*units::cm);
	cluster1_ave_pos_save = cluster1_ave_pos;
	cluster2_ave_pos = cluster_2->calc_ave_pos(p2,5*units::cm);
	cluster2_ave_pos_save = cluster2_ave_pos;

	if (num_dead_try==1){
	  dir1 = cluster_1->vhough_transform(cluster1_ave_pos,20*units::cm,1);
	  dir3 = cluster_2->vhough_transform(cluster2_ave_pos,20*units::cm,1);
	}else{
	  dir1 = cluster_1->vhough_transform(cluster1_ave_pos,80*units::cm,1);
	  dir3 = cluster_2->vhough_transform(cluster2_ave_pos,80*units::cm,1);
	}
	dir1_save = dir1;
	dir3_save = dir3;

	dir2.set(cluster2_ave_pos.x() - cluster1_ave_pos.x()+1e-9, cluster2_ave_pos.y() - cluster1_ave_pos.y()+1e-9, cluster2_ave_pos.z() - cluster1_ave_pos.z()+1e-9); // 2-1
	
      }else if (i==1){
	if (length_2 >= 15*units::cm &&(!(length_2 > 150*units::cm && dis<15*units::cm))){
	  cluster1_ave_pos = cluster1_ave_pos_save;//cluster_1->calc_ave_pos(p1,5*units::cm);
	  dir1 = dir1_save;//cluster_1->VHoughTrans(cluster1_ave_pos,80*units::cm);
	  
	  geo_vector_t dir_test(dir1);
	  dir_test = dir_test/dir_test.magnitude();
	  dir_test = (-1) * dir_test;
	  
	  std::pair<geo_point_t, double> temp_results = cluster_2->get_closest_point_along_vec(cluster1_ave_pos, dir_test, dis*2, 5*units::cm, 15, 10*units::cm);
	  
	  if (temp_results.second < 100*units::cm){
	    cluster2_ave_pos = cluster_2->calc_ave_pos(temp_results.first,5*units::cm);
	    dir3 = cluster_2->vhough_transform(cluster2_ave_pos,80*units::cm,1);
	    dir2.set(cluster2_ave_pos.x() - cluster1_ave_pos.x()+1e-9, cluster2_ave_pos.y() - cluster1_ave_pos.y()+1e-9, cluster2_ave_pos.z() - cluster1_ave_pos.z()+1e-9); // 2-1
	    
	  }else{
	    continue;
	  }
	}else{
	  continue;
	}
      }else if (i==2){
	if (length_2 >=15*units::cm&&(!(length_2 > 150*units::cm && dis<15*units::cm))){
	  cluster2_ave_pos = cluster2_ave_pos_save;//cluster_2->calc_ave_pos(p2,5*units::cm);
	  dir3 = dir3_save;//cluster_2->VHoughTrans(cluster2_ave_pos,80*units::cm);
	  
	  geo_point_t dir_test(dir3);
	  dir_test = dir_test / dir_test.magnitude();
	  dir_test = (-1) * dir_test;
	  
	  std::pair<geo_point_t, double> temp_results = cluster_1->get_closest_point_along_vec(cluster2_ave_pos, dir_test, dis*2, 5*units::cm, 15, 10*units::cm);
	  
	  if (temp_results.second < 100*units::cm){
	    cluster1_ave_pos = cluster_1->calc_ave_pos(temp_results.first,5*units::cm);
	    dir1 = cluster_1->vhough_transform(cluster1_ave_pos,80*units::cm,1);
	    dir2.set(cluster2_ave_pos.x() - cluster1_ave_pos.x()+1e-9, cluster2_ave_pos.y() - cluster1_ave_pos.y()+1e-9, cluster2_ave_pos.z() - cluster1_ave_pos.z()+1e-9); // 2-1
	    
	  }else{
	    continue;
	  }
	}else{
	  continue;
	}
      }
      
      geo_point_t ave_dir(cluster1_ave_pos.x()-cluster2_ave_pos.x(),cluster1_ave_pos.y()-cluster2_ave_pos.y(),cluster1_ave_pos.z()-cluster2_ave_pos.z());


      // use projection to deal with stuff ...
      if (fabs(ave_dir.angle(drift_dir)-3.1415926/2.)/3.1415926*180.>7.5){
      	// non-parallel case ...
      	if (WireCell::PointCloud::Facade::is_angle_consistent(dir1,dir2,false,10,angle_u,angle_v,angle_w,2)){
      	  if (length_2 < 8*units::cm&& WireCell::PointCloud::Facade::is_angle_consistent(dir1,dir2,false,5,angle_u,angle_v,angle_w,2))
      	    return true;
      	  if (length_2 < 15*units::cm && WireCell::PointCloud::Facade::is_angle_consistent(dir1,dir2,false,7.5,angle_u,angle_v,angle_w,2))
      	    return true;
      	  if (WireCell::PointCloud::Facade::is_angle_consistent(dir3,dir2,true,10,angle_u,angle_v,angle_w,2)){
      	    return true;
      	  }
      	}
      }
      
      
      double angle1 = (3.1415926-dir1.angle(dir2))/3.1415926*180.;
      double angle2 = dir3.angle(dir2)/3.1415926*180.;
      double angle3 = (3.1415926-dir1.angle(dir3))/3.1415926*180.;

      
      if (length_2 <=10*units::cm){
	if (angle1 < 15 && (angle2 < 60 || length_2 < 5*units::cm) ) return true;
      }else{
	if (angle1 < 15 && angle2 <15 && angle3 < 25 ||
	    angle3 < 10 && (angle1+angle2)<45 && dis < 5*units::cm )
	  return true;

	double ave_dis = sqrt(pow(cluster1_ave_pos.x()-cluster2_ave_pos.x(),2) + pow(cluster1_ave_pos.y()-cluster2_ave_pos.y(),2) + pow(cluster1_ave_pos.z()-cluster2_ave_pos.z(),2));
	geo_point_t test_point;
	double min_dis = 1e9, max_dis = -1e9;

	if (fabs(ave_dir.angle(drift_dir)-3.1415926/2.)/3.1415926*180. > 7.5  && ave_dis < 30*units::cm){
	  if (i==1){
	    for (int k=-5;k!=10;k++){
	      test_point.set(cluster1_ave_pos.x() - dir1.x() * (ave_dis +k*2*units::cm), cluster1_ave_pos.y() - dir1.y() * (ave_dis +k*2*units::cm), cluster1_ave_pos.z() - dir1.z() * (ave_dis +k*2*units::cm));
	      
	      auto temp_results = cluster_2->get_closest_point_mcell(test_point);
	      //reuse this
	      geo_point_t test_point1 = temp_results.first;
	      if (sqrt(pow(test_point1.x()-test_point.x(),2)+pow(test_point1.y()-test_point.y(),2)+pow(test_point1.z()-test_point.z(),2))<1.5*units::cm){
		double temp_dis = (test_point1.x() - cluster1_ave_pos.x())*dir1.x() + (test_point1.y() - cluster1_ave_pos.y())*dir1.y() + (test_point1.z() - cluster1_ave_pos.z())*dir1.z();
		temp_dis *=-1;
		if (temp_dis < min_dis) min_dis = temp_dis;
		if (temp_dis > max_dis) max_dis = temp_dis;
	      }
	    }

	    /* if (dis < 5*units::cm && fabs(length_1-110*units::cm)<2*units::cm && fabs(length_2-33*units::cm) < 2*units::cm) */
	    /*   std::cout << (max_dis - min_dis )/units::cm << std::endl; */
	    
	    if ((max_dis - min_dis)>2.5*units::cm) return true;
	  }else if (i==2){
	    for (int k=-5;k!=10;k++){
	      test_point.set(cluster2_ave_pos.x() - dir3.x() * (ave_dis +k*2*units::cm), cluster2_ave_pos.y() - dir3.y() * (ave_dis +k*2*units::cm),  cluster2_ave_pos.z() - dir3.z() * (ave_dis +k*2*units::cm));
	      
	      auto temp_results = cluster_1->get_closest_point_mcell(test_point);
	      //reuse this
	      geo_point_t test_point1 = temp_results.first;
	      if (sqrt(pow(test_point1.x()-test_point.x(),2)+pow(test_point1.y()-test_point.y(),2)+pow(test_point1.z()-test_point.z(),2))<1.5*units::cm){
		double temp_dis = (test_point1.x() - cluster2_ave_pos.x())*dir3.x() + (test_point1.y() - cluster2_ave_pos.y())*dir3.y() + (test_point1.z() - cluster2_ave_pos.z())*dir3.z();
		temp_dis *=-1;
		if (temp_dis < min_dis) min_dis = temp_dis;
		if (temp_dis > max_dis) max_dis = temp_dis;
	      }
	    }
	  // std::cout << cluster1->get_cluster_id() << " " << cluster2->get_cluster_id() << " " << min_dis/units::cm << " " << max_dis/units::cm << " " << length_1/units::cm << std::endl;
	  
	    /* if (dis < 5*units::cm && fabs(length_1-110*units::cm)<2*units::cm && fabs(length_2-33*units::cm) < 2*units::cm) */
	    /*   std::cout << (max_dis - min_dis )/units::cm << std::endl; */
	     
	    if ((max_dis - min_dis)>2.5*units::cm) return true;
	  }

	}
	
      }
    }
  }

  return false;
}
