#include "ros/ros.h"
#include <math.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <sstream>
#include <signal.h>
#include <boost/thread/thread.hpp>
#include <std_msgs/Bool.h>
//#include <villa_navi_service/GoTargetPos.h>
//#include <villa_navi_service/FastApproachAction.h> 
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/server/simple_action_server.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"

#define DYN_OFFSET_X 3.5
#define DYN_OFFSET_Y 3.5
#define X_MAPSIZE 20
#define MAP_RES 0.35

using namespace std;

class Obstacle_Checker
{
public:
    
  Obstacle_Checker(std::string name): 
  //as_(nh_, name, boost::bind(&Obstacle_Checker::executeCB, this,_1), false),
  //action_name_(name),
  IsGoal(false),
  IsRotated(false),
  IsActive(false),
  direction_z(1),
  split_size(3),
  srv_time(0.0)
  {
      cmd_velocity_pub= nh_.advertise<geometry_msgs::Twist>("/hsrb/command_velocity",10, true);
      dynamicmap_sub =nh_.subscribe<nav_msgs::OccupancyGrid>("/dynamic_obstacle_map_ref", 30, &Obstacle_Checker::dynamicmapCallback,this); 
      globalpose_sub=nh_.subscribe<geometry_msgs::PoseStamped>("/global_pose",10,&Obstacle_Checker::global_pose_callback,this);
      Scaled_dynamic_map_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/scaled_dynamic_map", 10, true);
      occ1_pub=nh_.advertise<std_msgs::Bool>("/obstacle2_Is_Occupied", 10, true);
      occ2_pub=nh_.advertise<std_msgs::Bool>("/obstacle3_Is_Occupied", 10, true);
      //m_client = nh_.serviceClient<villa_navi_service::GoTargetPos>("/navi_go_base");
      //target_sub=nh_.subscribe<geometry_msgs::PoseStamped>("/target_pose",10,&Obstacle_Checker::navtarget_callback,this);

    geometry_msgs::PointStamped input_point;
    input_point.header.frame_id="base_link";
    input_point.point.x=0.1;
    input_point.point.y=0.4;
    input_point.point.z=0.0;
    PointSet.push_back(input_point);
    input_point.header.frame_id="base_link";
    input_point.point.x=0.1;
    input_point.point.y=-0.4;
    input_point.point.z=0.0;
    PointSet.push_back(input_point);

    //input_point.header.frame_id="base_link";
    //input_point.point.x=0.3;
    //input_point.point.y=-0.1;
    //input_point.point.z=0.0;
    //PointSet.push_back(input_point);

    first_global_pose.resize(2,0.0);
    global_pose.resize(3,0.0);
    navtarget_pose.resize(2,0.0);
    //register the goal and feeback callbacks
    //as_.registerPreemptCallback(boost::bind(&Obstacle_Checker::preemptCB, this));
    //subscribe to the data topic of interest
    //as_.start();
    
  }

  ~Obstacle_Checker(void)
  {
  }

  //void preemptCB()
  //{
    //ROS_INFO("%s: Preempted", action_name_.c_str());
    //geometry_msgs::Twist vel_cmd;
                    //vel_cmd.linear.x=0.00;
                    //vel_cmd.linear.y=0.0;
                    //vel_cmd.linear.z=0.0;

                    //vel_cmd.angular.x=0.0;
                    //vel_cmd.angular.y=0.0;
                    //vel_cmd.angular.z=0.0;
                    //cmd_velocity_pub.publish(vel_cmd);
                    //IsActive=false;
                    //ROS_INFO("preempted called");

     //set the action state to preempted
    //as_.setPreempted();
  //}
    //feedback_.is_possible_go=true;
    // publish the feedback
    //as_.publishFeedback(feedback_);

    //result_.success=feedback_.is_possible_go;
    //as_.setSucceeded(result_);
    //as_set
    //ROS_INFO("%s: succeeded",action_name_.c_str());
    
  //}

  void global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {

      m_currentpose = msg->pose;

      global_pose[0]=msg->pose.position.x;
      global_pose[1]=msg->pose.position.y;

      tf::StampedTransform baselinktransform;
      listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
      listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
      double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

      global_pose[2]=yaw_tf;

      std::vector<bool> isObstacle(2,false);
      listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
      for(size_t point_idx(0);point_idx<PointSet.size();point_idx++)
      {
            geometry_msgs::PointStamped point_out;
            listener.transformPoint("map", PointSet[point_idx], point_out);
            ROS_INFO("transformed point : x : %.3lf, y : %.3lf ", point_out.point.x,point_out.point.y);
            int map_idx=Coord2CellNum(point_out.point.x,point_out.point.y);
            std::vector<double> front_point(2,0.0);
            Idx2Globalpose(map_idx,front_point);
            ROS_INFO("front point : x : %.3lf, y : %.3lf ",front_point[0], front_point[1]);

            if(Scaled_dynamic_map.data[map_idx]>1.0)
            {
                isObstacle[point_idx]=true;
                break;
            }
       }

      std_msgs::Bool IsObs_msg;
      IsObs_msg.data=isObstacle[0];
      occ1_pub.publish(IsObs_msg);

      IsObs_msg.data=isObstacle[1];
      occ2_pub.publish(IsObs_msg);



  }

  void dynamicmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	//ROS_INFO("I am calling");
	double small_pos_x, small_pos_y=0.0;
	double dist_x,dist_y=0.0;
	int map_coord_i,map_coord_j=0;
	int numcount=0;
	int	original_width=msg->info.width;			//140
	int	original_height= msg->info.height;		//140
	double original_x=msg->info.origin.position.x;
	double original_y=msg->info.origin.position.y;
	double oroginal_res=0.05;					//0.05

	//for static space map
	Scaled_dynamic_map.info.width=20;
	Scaled_dynamic_map.info.height= 20;
	Scaled_dynamic_map.info.resolution=0.35;
	//Scaled_dynamic_map.info.origin.position.x=global_pose[0]-DYN_OFFSET_X-0.5*Scaled_dynamic_map.info.resolution;
	//Scaled_dynamic_map.info.origin.position.y=global_pose[1]-DYN_OFFSET_Y-0.5*Scaled_dynamic_map.info.resolution;
	Scaled_dynamic_map.info.origin.position.x=global_pose[0]-DYN_OFFSET_X-0.0*Scaled_dynamic_map.info.resolution;
	Scaled_dynamic_map.info.origin.position.y=global_pose[1]-DYN_OFFSET_Y-0.0*Scaled_dynamic_map.info.resolution;
	Scaled_dynamic_map.data.resize(Scaled_dynamic_map.info.width*Scaled_dynamic_map.info.height);

   double base_origin_x =msg->info.origin.position.x;
   double base_origin_y =msg->info.origin.position.y;

	std::map<int,int> occupancyCountMap;
    int scaled_res=7;
    int map_idx=0;
    int scaled_result=0;

   for(int j(0);j<Scaled_dynamic_map.info.height;j++)
   	for(int i(0);i<Scaled_dynamic_map.info.width;i++)
   	{
   		map_idx=j*Scaled_dynamic_map.info.height+i;

   		//get global coordinate
   		double pos_x=i*Scaled_dynamic_map.info.resolution+Scaled_dynamic_map.info.origin.position.x+0.5*Scaled_dynamic_map.info.resolution;
   		double pos_y=j*Scaled_dynamic_map.info.resolution+Scaled_dynamic_map.info.origin.position.y+0.5*Scaled_dynamic_map.info.resolution;

   		numcount=0;
   		for(int k(0);k<scaled_res;k++)
   			for(int j(0);j<scaled_res;j++)
   			{
   				small_pos_x=pos_x+j*oroginal_res;
   				small_pos_y=pos_y+k*oroginal_res;
   				dist_x= small_pos_x-original_x;
				dist_y= small_pos_y-original_y;
				map_coord_i=floor(dist_x/oroginal_res);
				map_coord_j=floor(dist_y/oroginal_res);
				
				//static_map_ref_index
				int map_data_index=original_width*map_coord_j+map_coord_i;
				float temp_occupancy= msg->data[map_data_index];
    			 if(temp_occupancy>0)
				 	numcount++;
   			}

   			if(numcount>15)
   				scaled_result=60;
   			else
   				scaled_result=0;

               Scaled_dynamic_map.data[map_idx]=scaled_result;
   	}

     //find index from
	 Scaled_dynamic_map.header.stamp =  ros::Time::now();
	 Scaled_dynamic_map.header.frame_id = "map"; 
     Scaled_dynamic_map_pub.publish(Scaled_dynamic_map);
}
    void Idx2Globalpose(int idx, std::vector<double>& global_coord)
    {
        global_coord.resize(2,0.0);

        int res = (int) (idx/ Scaled_dynamic_map.info.width);
        int div = (int) (idx%Scaled_dynamic_map.info.width);

        global_coord[0]=res*Scaled_dynamic_map.info.resolution+Scaled_dynamic_map.info.origin.position.x;
        global_coord[1]=div*Scaled_dynamic_map.info.resolution+Scaled_dynamic_map.info.origin.position.y;
    }

    bool process_target(double goal_x, double goal_y,double goal_theta){


        float diff_x=goal_x-global_pose[0];
        float diff_y=goal_y-global_pose[1];
        float dist =sqrt(pow(diff_x,2)+pow(diff_y,2));

        if(dist>2.5)
            split_size=5;
        else if(dist<0.75)
            split_size=0;
        else
            split_size=3;

        double coeff=(double)(1/(double)split_size);
    
        //ROS_INFO("diff x : %.3lf , y : %.3lf ",diff_x,diff_y);
        navtarget_pose.resize(3,0.0);
        navtarget_pose[0]=global_pose[0]+diff_x;
        navtarget_pose[1]=global_pose[1]+diff_y;
        
        //ROS_INFO("global theta : %.3lf, navigation_theta",global_pose[2],navtarget_pose[2]);
        //ROS_INFO("nglobal thetax : %.3lf  ",global_pose[2]);
        //ROS_INFO("target x : %.3lf , y : %.3lf ",navtarget_pose[0],navtarget_pose[1]);
        geometry_msgs::Vector3Stamped gV, tV;

        gV.vector.x=navtarget_pose[0]-global_pose[0];
        gV.vector.y=navtarget_pose[1]-global_pose[1];
        gV.vector.z=0.5;

        gV.header.stamp-ros::Time();
        gV.header.frame_id="/map";
        listener.transformVector("/base_link",gV,tV);

        double target_angle_baselink= atan2(tV.vector.y,tV.vector.x);
        ROS_INFO("target_angle_baselink: %.3lf", target_angle_baselink);
        navtarget_pose[2]=(target_angle_baselink)+global_pose[2];
        ROS_INFO("nav target_angle: %.3lf, global_pose : %.3lf", navtarget_pose[2], global_pose[2]);
        //ToDo : origientation (x,ytheta)
        double diff_theta=navtarget_pose[2]-global_pose[2];
        if(diff_theta>M_PI)
            diff_theta=diff_theta-2*M_PI;
        else if (diff_theta < (-1 * M_PI))
            diff_theta=diff_theta+2*M_PI;


        ROS_INFO("diff_theta: %.3lf", diff_theta);

        if(diff_theta>0)
            direction_z=1;
        else
            direction_z=-1;


        
        checklistmap.clear();
        for(size_t k(0);k<split_size;k++)
        {
            double temp_x=global_pose[0]+k*(coeff)*diff_x;
            double temp_y=global_pose[1]+k*(coeff)*diff_y;
            int map_idx=Coord2CellNum(temp_x,temp_y);
            
            if(Scaled_dynamic_map.data[map_idx]>10)
            { 
                checklistmap[map_idx]=false;
                //ROS_INFO("%d, false",k);
                
            }
            else{
                checklistmap[map_idx]=true;
                //ROS_INFO("%d, true",k);
            }
            
            checklistbool.push_back(checklistmap[map_idx]);

        }

        int count=0;
   
        return true;
    }

    int Coord2CellNum(double _x, double _y)
    {	
        //ROS_INFO("x: %.2lf, y: %.2lf", _x, _y);
        std::vector<int> target_Coord;
        target_Coord.resize(2,0);

        double reference_origin_x;
        double reference_origin_y;

        reference_origin_x=Scaled_dynamic_map.info.origin.position.x;
        reference_origin_y=Scaled_dynamic_map.info.origin.position.y;

        double  temp_x  = _x-reference_origin_x;
        double  temp_y = _y-reference_origin_y;

        target_Coord[0]= (int) (temp_x/MAP_RES);
        target_Coord[1]= (int)(temp_y/MAP_RES);

        //ROS_INFO("targertcoord: x: %d, y: %d", target_Coord[0], target_Coord[1]);

        std::vector<int> dynamicCoord;
        dynamicCoord.resize(2);
        dynamicCoord[0]=Scaled_dynamic_map.info.origin.position.x+Scaled_dynamic_map.info.resolution*target_Coord[0]+0.5*Scaled_dynamic_map.info.resolution;
        dynamicCoord[1]=Scaled_dynamic_map.info.origin.position.y+Scaled_dynamic_map.info.resolution*target_Coord[1]+0.5*Scaled_dynamic_map.info.resolution;

        int index= target_Coord[0]+Scaled_dynamic_map.info.width*target_Coord[1];
        //ROS_INFO("dynamic_map idx : %d", index);
        return index;
    }


    bool CheckRotation(double cur_angle, double desired_angle, double err_criterion)
    {
        double temp_dist=0.0;

        temp_dist=pow((cur_angle-desired_angle),2);
        temp_dist=sqrt(temp_dist);
        //ROS_INFO("temp_yaw_diff: %.4lf ", temp_dist);

        double temp_criterion =sqrt(pow((temp_dist-err_criterion),2));
        //ROS_INFO("temp_yaw_error: %.4lf ", temp_criterion);

        if(temp_criterion<err_criterion)
            return true;

        return false;
    }



    bool checkDistance(std::vector<double>& pos1, std::vector<double>& pos2, double err_criterion)
    {
        if((pos1.size()<1) || (pos2.size()<1))
        {
            ROS_INFO("vector size wrong");
            return false;
        }
        double temp_dist=0.0;
        for(size_t i(0);i<2;i++) 
            temp_dist+=pow((pos1[i]-pos2[i]),2);

        temp_dist=sqrt(temp_dist);

        //ROS_INFO("temp_dist: %.4lf ", temp_dist);
        double temp_criterion =sqrt(pow((temp_dist-err_criterion),2));

        //ROS_INFO("distance error: %.4lf ", temp_criterion);
        if(temp_criterion<err_criterion)
            return true;

        return false;
    }

  


  
protected:
    
  ros::NodeHandle nh_;
  int data_count_, goal_;
  float sum_, sum_sq_;
  ros::Subscriber globalpose_sub;
  ros::Subscriber target_sub;
  ros::Subscriber dynamicmap_sub;
  ros::Publisher cmd_velocity_pub;
  ros::Publisher occ1_pub;
  ros::Publisher occ2_pub;
  ros::Publisher Scaled_dynamic_map_pub;
  int direction_z;

  geometry_msgs::Pose m_currentpose;
  nav_msgs::OccupancyGrid Scaled_dynamic_map;
  tf::TransformListener   listener;

  std::vector<double> navtarget_pose;
  std::vector<double> first_global_pose;
  std::vector<double> global_pose;
  //std::vector<int> checklist_mapidxset;
  std::map<int,bool> checklistmap;
  std::vector<bool> checklistbool;
  std::vector<geometry_msgs::PointStamped> PointSet;
  ros::ServiceClient m_client;
  int split_size;
  bool IsGoal;
  bool IsRotated;
  bool IsActive;
  double srv_time; 


  //geometry_msgs::PoseArray checklistposes;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_manager");
  Obstacle_Checker obs_manager(ros::this_node::getName());
  double ros_rate = 2;
  ros::Rate r(ros_rate);

  while (ros::ok())
  {
	 ros::spinOnce();
     r.sleep();
  }


  ros::spin();

  return 0;
}
