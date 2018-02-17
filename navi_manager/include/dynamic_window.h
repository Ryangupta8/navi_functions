#include <vector>
#include <fstream>
#include <string>
#include <map>
#include <stdint.h>
#include "ros/ros.h"
#include <ros/package.h>
#include <eigen3/Eigen/Dense>
#include <MapParam.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "srBSpline.h"
#include <sensor_msgs/JointState.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <tf/tf.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include <tf/transform_datatypes.h>
#include <cmath>
#include <cfloat>

#define FREE_CELL 0
#define St_OBS_CELL 1
#define Dy_OBS_CELL 1
#define Human_CELL 3

#define Start_X 1
#define Start_Y 1
//80 20
#define Goal_X 2
#define Goal_Y 2

#define DYN_OFFSET_X 3.5
#define DYN_OFFSET_Y 3.5

#define LASER_range_person  2.5
#define YOLO_range_person   4.0

#define ra (-1.0)
#define FOVW 40       //field of view width
#define MATH_PI 3.14159265359


using namespace Eigen;
using namespace std;


class Dynamic_window
{
 public:
 	Dynamic_window(MapParam* _pMapParam);
 	Dynamic_window(){}
 	~Dynamic_window();

 	MapParam* 	pMapParam;
 	tf::TransformListener 	  listener;
 	int               X_mapSize;
 	int               Y_mapSize;
 	int               Num_Grids;
	vector<int>		  cell_xy;
 	vector<int>       m_localoccupancy;
 	vector<int>       m_dynamic_occupancy;
 	int  			  num_of_detected_human_yolo;
 	vector<double>    filtered_target;

 	//human sets
    std::vector< std::vector< double > > cur_people;
    std::vector<int> human_occupied_idx;
	std::vector<int> human_occupied_leg_idx;
	std::vector<int> visiblie_idx_set;
	std::vector<double> Head_Pos; 	
	std::vector<double> global_pose;
 	std::vector<double> Map_orig_Vector;
	std::vector<double> CurVector;
    
 	int Local_X_start;
 	int Local_Y_start;
    int dyn_path_num;
 	vector<int>  MDPPath;
 	vector<int>  Dyn_MDPPath;
 	srBSpline*          m_Spline;
 	srBSpline*          m_CubicSpline_x;
 	srBSpline*          m_CubicSpline_y;


	ros::NodeHandle  m_node;
	ros::Publisher   Scaled_static_map_pub;
	ros::Publisher   Scaled_dynamic_map_pub;
	ros::Publisher   Scaled_dynamic_map_path_pub;
	ros::Publisher   Path_Pub;
	ros::Publisher   camera_map_pub;
  
	//Static_mdp
	int  scaling=12;
	nav_msgs::OccupancyGrid camera_map;
	nav_msgs::OccupancyGrid Scaled_static_map;
	nav_msgs::OccupancyGrid Scaled_dynamic_map;
	nav_msgs::OccupancyGrid Scaled_static_map_path;
	nav_msgs::OccupancyGrid Scaled_dynamic_map_path;
	nav_msgs::OccupancyGrid Human_Belief_Scan_map;
	nav_msgs::Path Pre_dynamicSplinePath;
	nav_msgs::Path path;
	
	//functions
 	void 			Init();								 //Initialize function
 	void 			setPMapParam(MapParam* _pMapParam);
 	void 			CellNum2Coord(const int Cell_idx, vector<int>& cell_xy);
 	int  			Coord2CellNum(vector<int> cell_xy);
 	vector<int>     Global2LocalCoord(vector<int> Global_coord);
 	bool 			getlinevalue(int line_type,double input_x, double input_y);
    void            mdppath_callback(const nav_msgs::Path::ConstPtr & msg);
 	void 			joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg);
 	void 			static_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
 	void 			dynamic_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
 	void			ClikedpointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
 	void			Human_MarkerCallback(const visualization_msgs::Marker::ConstPtr& msg);
 	void			Human_MarkerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void 			global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void 			Global2MapCoord(const vector<double>& _globalcoord, vector<int>& MapCoord);
    void 			CellNum2globalCoord(const int Cell_idx, std::vector<double>& cell_xy);
	int 			CoordinateTransform_Global2_cameraMap(float global_x, float global_y);
  	bool 			check_cameraregion(float x_pos,float y_pos);
	void            Mapcoord2GlobalCoord(const vector<int>& _Mapcoord, vector<double>& GlobalCoord);
	void 			Mapcoord2DynamicCoord(const vector<int>& _Mapcoord, vector<double>& dynamicCoord);
	double			getdistance(vector<double> cur, vector<double> goal);
	double 			getDistance_from_Vec(std::vector<double> origin, double _x, double _y);
	bool            IsinDynamicMap(float global_x, float global_y);
	bool 			Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2,double criterion);void 			setDesiredHeading(double _heading);
	void 			getCameraregion();
	bool 			NotUpdatedCameraregion(int idx);
	//Publish
	//void 			publishpaths();
  	void 			publish_cameraregion();
    //void 			Publish_dynamicPath();
    
};

