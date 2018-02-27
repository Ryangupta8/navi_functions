#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <navi_service/GoTargetPos.h>
using namespace Eigen;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "navi_srvice_client");

  ros::NodeHandle n;
         
  double door_x_map = 3.0;
  double door_y_map = 2.0;
  
  //getparamters from ros
  //if(n.getParam("nav_srv_node/door_location_x",door_x_map))
  //{
      //std::cout<<"get parameter succeed"<< door_x_map <<std::endl;
  //}
  //if(n.getParam("villa_nav_srv_node/door_location_y",door_y_map))
  //{
      //std::cout<<"get parameter succeed: "<< door_y_map<<std::endl;
  //}
  
  ros::Rate loop_rate(50);

  ros::ServiceClient service_client = n.serviceClient<navi_service::GoTargetPos>("/navi_go_base");
  

  std::cout<<"navi service target x:  "<< door_x_map <<std::endl;
  std::cout<<"navi service target y: "<< door_y_map <<std::endl;

  navi_service::GoTargetPos navi_srv;
  navi_srv.request.x_from_map=door_x_map;
  navi_srv.request.y_from_map=door_y_map;
  navi_srv.request.theta_from_map=0;

  service_client.call(navi_srv);
	   

  return 0;
}




