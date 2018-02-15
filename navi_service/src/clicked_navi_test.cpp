#include "ros/ros.h"
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <string>
#include <map>
#include <sstream>
#include <signal.h>
#include <boost/thread/thread.hpp>
// #include <stdint.h>

#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <sensor_msgs/JointState.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <navi_service/GoTargetPos.h>

using namespace Eigen;


class click_navi_manager{

public:
  explicit click_navi_manager(){}
  ~click_navi_manager(){}

  ros::Publisher Gaze_point_pub;
  ros::Publisher Gaze_activate_pub;
  ros::Publisher setNavTarget_pub;
  
  tf::TransformListener     listener;

  void Publish_nav_target(float _x, float _y, float _theta)
  {

      ROS_INFO("x : %.3lf , y : %.3lf", _x,_y);
    
      move_base_msgs::MoveBaseActionGoal Navmsgs;
      Navmsgs.header.stamp =  ros::Time::now();
      Navmsgs.goal.target_pose.header.frame_id = "map";

      Navmsgs.goal.target_pose.pose.position.x=_x;
      Navmsgs.goal.target_pose.pose.position.y=_y;

      //Todo : theta >> quternian
      Navmsgs.goal.target_pose.pose.position.z=0.0;
      Navmsgs.goal.target_pose.pose.orientation.x=0.0;
      Navmsgs.goal.target_pose.pose.orientation.y=0.0;
      Navmsgs.goal.target_pose.pose.orientation.z=0.0;
      Navmsgs.goal.target_pose.pose.orientation.w=1.0;

      // setViewpointTarget(leg_target);
      setNavTarget_pub.publish(Navmsgs);
      ROS_INFO("navgation published");


  }
 
  // void setViewpointTarget(const std::vector<double> pos);
  void ClikedpointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
      double x_map=msg->point.x;
      double y_map=msg->point.x;
      double theta_map=0.0;      
      // printf("Receive point %.3lf , %.3lf \n",x_map,y_map);
    

      Publish_nav_target(x_map,y_map,theta_map);
  }

};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "manager");

  click_navi_manager manager;
    
  ros::NodeHandle n;
  ros::Subscriber clicked_point_sub;
  
  clicked_point_sub = n.subscribe<geometry_msgs::PointStamped>("/clicked_point", 10, &click_navi_manager::ClikedpointCallback,&manager);
  manager.setNavTarget_pub=n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/move/goal",50,true);

  ros::Rate loop_rate(50);

  double ros_rate = 3.0;
  ros::Rate r(ros_rate);

  while (ros::ok())
  {	   
     // manager.Publish_nav_target();
     ros::spinOnce();
     r.sleep();
  }

  ros::spin();

  return 0;
}




