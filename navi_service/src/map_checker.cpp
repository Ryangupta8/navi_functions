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
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/server/simple_action_server.h>
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"

#define DYN_OFFSET_X 3.5
#define DYN_OFFSET_Y 3.5
#define X_MAPSIZE 20
#define MAP_RES 0.05

using namespace std;

class Map_Checker
{
public:
    
  Map_Checker(std::string name): 
  //as_(nh_, name, boost::bind(&Map_Checker::executeCB, this,_1), false),
  //action_name_(name),
  IsGoal(false),
  IsRotated(false),
  IsActive(false),
  Map_Recieved(false),
  direction_z(1),
  split_size(3),
  srv_time(0.0)
  {
      cmd_velocity_pub= nh_.advertise<geometry_msgs::Twist>("/hsrb/command_velocity",10, true);
      dynamicmap_sub =nh_.subscribe<nav_msgs::OccupancyGrid>("/dynamic_obstacle_map_ref", 30, &Map_Checker::dynamicmapCallback,this); 
      globalpose_sub=nh_.subscribe<geometry_msgs::PoseStamped>("/global_pose",10,&Map_Checker::global_pose_callback,this);
      //Scaled_dynamic_map_pub=nh_.advertise<nav_msgs::OccupancyGrid>("/scaled_dynamic_map", 10, true);
      occ1_pub=nh_.advertise<std_msgs::Bool>("/obstacle2_Is_Occupied", 10, true);
      occ2_pub=nh_.advertise<std_msgs::Bool>("/obstacle3_Is_Occupied", 10, true);

    geometry_msgs::PointStamped input_point;
    input_point.header.frame_id="base_link";
    input_point.point.x=0.01;
    input_point.point.y=0.75;
    input_point.point.z=0.0;
    PointSet.push_back(input_point);
    input_point.point.x=0.00;
    input_point.point.y=-0.75;
    input_point.point.z=0.0;
    PointSet.push_back(input_point);

    //input_point.header.frame_id="base_link";
    //input_point.point.x=0.01;
    //input_point.point.y=-0.5;
    //input_point.point.z=0.0;
    //PointSet.push_back(input_point);
    //input_point.point.x=0.01;
    //input_point.point.y=-1.0;
    //input_point.point.z=0.0;
    //PointSet.push_back(input_point);

    //input_point.header.frame_id="base_link";
    //input_point.point.x=0.3;
    //input_point.point.y=-0.1;
    //input_point.point.z=0.0;
    //PointSet.push_back(input_point);

    first_global_pose.resize(2,0.0);
    global_pose.resize(3,0.0);
    navtarget_pose.resize(2,0.0);
    //register the goal and feeback callbacks
    //as_.registerPreemptCallback(boost::bind(&Map_Checker::preemptCB, this));
    //subscribe to the data topic of interest
    //as_.start();
    
  }

  ~Map_Checker(void)
  {
  }

  void global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
      m_currentpose = msg->pose;

      global_pose[0]=msg->pose.position.x;
      global_pose[1]=msg->pose.position.y;

      tf::StampedTransform baselinktransform;
      listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
      listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
      double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

      global_pose[2]=yaw_tf;

      if(!Map_Recieved)
          return;
     //check obstacles at two sides of the robot, pointset has two points defined w.r.t the base link frame
      std::vector<bool> isObstacle(2,false);
      listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
      for(size_t point_idx(0);point_idx<PointSet.size();point_idx++)
      {
            geometry_msgs::PointStamped point_out;
            listener.transformPoint("map", PointSet[point_idx], point_out);
            isObstacle[point_idx]=check_obstacle(point_out.point.x, point_out.point.y);
      }

      bool result_obs=false;
      //if(isObstacle[0] || isObstacle[1])
      if(isObstacle[0])
          result_obs=true;
      else
          result_obs=false;

      std_msgs::Bool IsObs_msg;
      IsObs_msg.data=result_obs;
      occ1_pub.publish(IsObs_msg);

      result_obs=false;
      //if(isObstacle[2] || isObstacle[3])
      if(isObstacle[1])
          result_obs=true;
      else
          result_obs=false;

      std_msgs::Bool IsObs_msg2;
      IsObs_msg2.data=result_obs;
      occ2_pub.publish(IsObs_msg2);

  }

  void dynamicmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
  {
    dynamic_map=*msg;
    Map_Recieved=true;
  }

    void Idx2Globalpose(int idx, std::vector<double>& global_coord)
    {
        global_coord.resize(2,0.0);

        int res = (int) (idx/dynamic_map.info.width);
        int div = (int) (idx%dynamic_map.info.width);

        global_coord[0]=res*dynamic_map.info.resolution+dynamic_map.info.origin.position.x;
        global_coord[1]=div*dynamic_map.info.resolution+dynamic_map.info.origin.position.y;
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
            
            if(dynamic_map.data[map_idx]>10)
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
    bool check_obstacle(double _x, double _y)
    {
        int map_idx=Coord2CellNum(_x,_y);

        if(dynamic_map.data[map_idx]>20)
        {
            return true;
        }
        else{
            return false;
        }

    }

    int Coord2CellNum(double _x, double _y)
    {	
        //ROS_INFO("x: %.2lf, y: %.2lf", _x, _y);
        std::vector<int> target_Coord;
        target_Coord.resize(2,0);

        double reference_origin_x;
        double reference_origin_y;

        reference_origin_x=dynamic_map.info.origin.position.x;
        reference_origin_y=dynamic_map.info.origin.position.y;

        double  temp_x  = _x-reference_origin_x;
        double  temp_y = _y-reference_origin_y;

        target_Coord[0]= (int) (temp_x/dynamic_map.info.resolution);
        target_Coord[1]= (int)(temp_y/dynamic_map.info.resolution);

        //ROS_INFO("targertcoord: x: %d, y: %d", target_Coord[0], target_Coord[1]);

        std::vector<int> dynamicCoord;
        dynamicCoord.resize(2);
        dynamicCoord[0]=dynamic_map.info.origin.position.x+dynamic_map.info.resolution*target_Coord[0]+0.5*dynamic_map.info.resolution;
        dynamicCoord[1]=dynamic_map.info.origin.position.y+dynamic_map.info.resolution*target_Coord[1]+0.5*dynamic_map.info.resolution;

        int index= target_Coord[0]+dynamic_map.info.width*target_Coord[1];
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
  int direction_z;

  geometry_msgs::Pose m_currentpose;
  nav_msgs::OccupancyGrid dynamic_map;
  //nav_msgs::OccupancyGrid dynamic_map;
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
  bool Map_Recieved;
  double srv_time; 


  //geometry_msgs::PoseArray checklistposes;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_manager");
  Map_Checker obs_manager(ros::this_node::getName());
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
