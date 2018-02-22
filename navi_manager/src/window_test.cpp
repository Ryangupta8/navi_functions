#include "ros/ros.h"
#include "dynamic_window.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
// #include <Eigen/Dense>
#include <sstream>
#include <boost/thread/thread.hpp>
#include "srBSpline.h"

using namespace Eigen;

//MapParam   dynamicmapParam(10,10,0.75);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_window");
  
  Dynamic_window window_manager; 
  MapParam   dynamicmapParam(5,5,1.0);
  window_manager.setPMapParam(&dynamicmapParam);

  // ros::Rate r(5);
  ros::Subscriber Point_sub;          //subscriber clicked point
  ros::Subscriber SplinePath_sub;
  ros::Subscriber dynamicmap_sub;
  ros::Subscriber global_pos_sub;
  ros::Subscriber jointstates_sub;
  ros::Subscriber  human_yolo_sub;
  ros::NodeHandle n;
  
  dynamicmap_sub   = n.subscribe<nav_msgs::OccupancyGrid>("/dynamic_obstacle_map_ref", 30, &Dynamic_window::dynamic_mapCallback,&window_manager); 
  jointstates_sub  = n.subscribe<sensor_msgs::JointState>("/hsrb/joint_states", 10, &Dynamic_window::joint_states_callback,&window_manager);
  global_pos_sub   = n.subscribe<geometry_msgs::PoseStamped>("/global_pose", 10, &Dynamic_window::global_pose_callback,&window_manager);
  SplinePath_sub=  n.subscribe<nav_msgs::Path>("/mdp_path", 10, &Dynamic_window::mdppath_callback,&window_manager);
  //human_yolo_sub   = n.subscribe<visualization_msgs::MarkerArray>("/human_boxes", 10, &Dynamic_window::Human_MarkerArrayCallback,&window_manager);

  ros::Rate loop_rate(30);
  while (ros::ok())
  {

     //window_manager.set_dynamicPath();     
     window_manager.publish_cameraregion();
   
  	 ros::spinOnce();
     loop_rate.sleep();  
  }

  ros::spin();

  return 0;
}




