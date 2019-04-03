#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <cstring>
#include <map>
#include <stdint.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <Eigen/Dense>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <villa_navi_service/Approach.h>
#include <XmlRpcValue.h>
#include <yaml-cpp/yaml.h>

using namespace Eigen;
using namespace std;

#define DYN_OFFSET_X 3.5
#define DYN_OFFSET_Y 3.5
#define X_MAPSIZE 14
#define MAP_RES 0.5

class ApproachManager{

public:
	explicit ApproachManager(ros::NodeHandle n_);
	~ApproachManager();


	ros::Publisher vel_pub;
	ros::Subscriber globalpose_sub;
	ros::Subscriber dynamicmap_sub;
	ros::Publisher  Scaled_dynamic_map_pub;
	ros::ServiceServer m_service;
	std::map< std::string, std::vector<double> > goal_maps;
		
	int targetup;
    tf::TransformListener 	  listener;
    geometry_msgs::Pose m_currentpose;
	nav_msgs::OccupancyGrid Scaled_dynamic_map;
	bool IsGoal;
	bool IsActive;

	std::vector<double> Robot_Pos;				//x,y,theta
	std::vector<double> Head_Pos;				//x,y,theta
	std::vector<double> global_pose;
	std::vector<double> first_global_pose;
	std::vector<double> target_dist;

	void Sending_velcmd();
	void Publish_nav_target(float x_, float y_, float t_);
	bool goTarget(villa_navi_service::Approach::Request &req, villa_navi_service::Approach::Response &res);	
	bool comparetwoposes(std::vector<double>& pos1, std::vector<double>& pos2, double criterion);
    void global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void dynamicmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    int Coord2CellNum(double _x, double _y);

};
