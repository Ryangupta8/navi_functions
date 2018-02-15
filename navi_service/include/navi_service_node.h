#include <vector>
#include <fstream>
#include <string>
#include <map>
#include <stdint.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <navi_service/GoTargetPos.h>



class villa_navi_srv{

public:
	villa_navi_srv();
	~villa_navi_srv();

	ros::Publisher Gaze_point_pub;
	ros::Publisher Gaze_activate_pub;
	ros::Publisher setNavTarget_pub;
	
	tf::TransformListener 	  listener;

	// void Init_parameters();
	void InitializeBelief();
	void Publish_nav_target(float _x, float _y, float _theta);
	void setViewpointTarget(const std::vector<double> pos);
	bool goTarget(navi_service::GoTargetPos::Request &req, navi_service::GoTargetPos::Response &res);	
 	void ClikedpointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);

};
