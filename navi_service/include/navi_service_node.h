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

	// std::vector<double> Robot_Pos;				//x,y,theta
	// std::vector<double> Head_Pos;				//x,y,theta
	// std::vector<double> global_pose;
	// std::vector<double> Human_target_candidate;				//x,y,theta
	// std::vector<double> Track_human_target;				//x,y,theta
	// std::vector<double> leg_target;				//x,y,theta
	// std::vector< std::vector< double > > Last_detected_human;
	// std::vector< std::vector< double > > Cur_detected_human;
	// std::vector< std::vector< double > > Cur_existed_human;
	// std::vector< std::vector< double > > Cur_leg_human;
	// std::vector< std::vector< double > > Cur_leg_yolo_human;
	// std::vector< double >  viewpoint_robot;
	// std::vector<double> angle_people_set;
	// std::vector<double> angle_criticalvalue;

	// std::vector<int> human_occupied_idx;
	// std::vector<int> human_occupied_leg_idx;
	// std::vector<int> visiblie_idx_set;

	// std::vector<int> index_of_human_occ_cells_updated_recently;
	// std::map<int, float> map_index_of_human_cells_to_prob;

	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
	
	// nav_msgs::OccupancyGrid dynamic_belief_map;
	// nav_msgs::OccupancyGrid human_belief_map;
	// nav_msgs::OccupancyGrid static_belief_map;
	// std_msgs::Int8 track_cmd;
	// visualization_msgs::MarkerArray human_boxes_array;
	// bool OnceTargeted;


	// void Init_parameters();
	void InitializeBelief();
	void Publish_nav_target(float _x, float _y, float _theta);
	void setViewpointTarget(const std::vector<double> pos);
	bool goTarget(navi_service::GoTargetPos::Request &req, navi_service::GoTargetPos::Response &res);	

	// std::vector<double> m_dyn_occupancy;
	// std::vector<double> m_prob_occupancy;
	// std::vector<int>    m_yolo_idx_set;
	// std::vector<int>    m_leg_idx_set;
	// std::vector<double> m_human_posx;
	// std::vector<double> m_human_posy;


	// int pub_iters;
	// int detect_iters;
	// double Camera_angle;
	// int print_iter;

};
