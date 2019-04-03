#include "approach_server_node.h"


ApproachManager::ApproachManager(ros::NodeHandle n_){

  IsGoal=false;
  IsActive=false;
  first_global_pose.resize(2,0.0);
  target_dist.resize(2,0.0);
  global_pose.resize(3,0.0);

  vel_pub = n_.advertise<geometry_msgs::Twist>("/hsrb/command_velocity",10, true);
  Scaled_dynamic_map_pub=n_.advertise<nav_msgs::OccupancyGrid>("/scaled_dynamic_map", 10, true);
  dynamicmap_sub =n_.subscribe<nav_msgs::OccupancyGrid>("/dynamic_obstacle_map_ref", 30, &ApproachManager::dynamicmapCallback,this); 
  globalpose_sub=n_.subscribe<geometry_msgs::PoseStamped>("/global_pose",10,&ApproachManager::global_pose_callback,this);
  m_service = n_.advertiseService("/approach_go",  &ApproachManager::goTarget,this);

}
ApproachManager::~ApproachManager(){}


bool ApproachManager::goTarget(villa_navi_service::Approach::Request &req, villa_navi_service::Approach::Response &res)
{
    first_global_pose[0]=global_pose[0];
    first_global_pose[1]=global_pose[1];

    target_dist[0]=req.x_from_base;
    target_dist[1]=req.y_from_base;

    IsActive=true;
    IsGoal=false;
    ROS_INFO("Approach service is called!!");

    res.is_possible_go = true;
}

bool ApproachManager::comparetwoposes(std::vector<double>& pos1, std::vector<double>& pos2, double criterion)
{


    if((pos1.size()<1) || (pos2.size()<1))
    {
        ROS_INFO("vector size wrong");
        return false;
    }

  double temp_dist=0.0;
  for(size_t i(0);i<2;i++) 
  {
    temp_dist+=pow((pos1[i]-pos2[i]),2);
  }

  temp_dist=sqrt(temp_dist);
  ROS_INFO("temp_dist: %.4lf ", temp_dist);
  
  double temp_criterion =pow((temp_dist-criterion),2);
  double err_criterion= 0.02;

  ROS_INFO("error: %.4lf ", temp_criterion);
  
  if(temp_criterion<err_criterion)
      return true;

  return false;

}

void ApproachManager::global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

  m_currentpose = msg->pose;

   global_pose[0]=msg->pose.position.x;
   global_pose[1]=msg->pose.position.y;

   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
   listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

    global_pose[2]=yaw_tf;
}


void ApproachManager::dynamicmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
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
	Scaled_dynamic_map.info.width=14;
	Scaled_dynamic_map.info.height= 14;
	Scaled_dynamic_map.info.resolution=0.5;
	Scaled_dynamic_map.info.origin.position.x=global_pose[0]-DYN_OFFSET_X-0.5*Scaled_dynamic_map.info.resolution;
	Scaled_dynamic_map.info.origin.position.y=global_pose[1]-DYN_OFFSET_Y-0.5*Scaled_dynamic_map.info.resolution;
	Scaled_dynamic_map.data.resize(Scaled_dynamic_map.info.width*Scaled_dynamic_map.info.height);

   double base_origin_x =msg->info.origin.position.x;
   double base_origin_y =msg->info.origin.position.y;

	std::map<int,int> occupancyCountMap;
    int scaled_res=10;
    int map_idx=0;
    int scaled_result=0;

   for(int j(0);j<Scaled_dynamic_map.info.height;j++)
   	for(int i(0);i<Scaled_dynamic_map.info.width;i++)
   	{
   		map_idx=j*Scaled_dynamic_map.info.height+i;

   		//get global coordinate
   		double pos_x=i*Scaled_dynamic_map.info.resolution+Scaled_dynamic_map.info.origin.position.x;
   		double pos_y=j*Scaled_dynamic_map.info.resolution+Scaled_dynamic_map.info.origin.position.y;

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

   			if(numcount>25)
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

void ApproachManager::Sending_velcmd()
{
    if(IsActive)
    {
        if(!IsGoal)
        {
            if(comparetwoposes(global_pose,first_global_pose,target_dist[0]))
            {
                IsGoal=true;
                IsActive=false;
                return;
                //break;
            }

            geometry_msgs::Twist vel_cmd;
            vel_cmd.linear.x=0.05;
            vel_cmd.linear.y=0.0;
            vel_cmd.linear.z=0.0;

            vel_cmd.angular.x=0.0;
            vel_cmd.angular.y=0.0;
            vel_cmd.angular.z=0.0;
            vel_pub.publish(vel_cmd);
        }
    }
}

int ApproachManager::Coord2CellNum(double _x, double _y)
{	
    ROS_INFO("x: %.2lf, y: %.2lf", _x, _y);
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
 
    ROS_INFO("targertcoord: x: %d, y: %d", target_Coord[0], target_Coord[1]);

    std::vector<int> dynamicCoord;
	dynamicCoord.resize(2);
	dynamicCoord[0]=Scaled_dynamic_map.info.origin.position.x+Scaled_dynamic_map.info.resolution*target_Coord[0]+0.5*Scaled_dynamic_map.info.resolution;
	dynamicCoord[1]=Scaled_dynamic_map.info.origin.position.y+Scaled_dynamic_map.info.resolution*target_Coord[1]+0.5*Scaled_dynamic_map.info.resolution;

	int index= target_Coord[0]+14*target_Coord[1];
    ROS_INFO("dynamic_map idx : %d", index);
	return index;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "approach_server_node");

  ros::NodeHandle n;
  ApproachManager approach_manager(n);
    
  ros::Rate loop_rate(50);

   double ros_rate = 3.0;
  ros::Rate r(ros_rate);

  while (ros::ok())
  {

     approach_manager.Sending_velcmd();
	 ros::spinOnce();
     r.sleep();
  }

  ros::spin();

  return 0;
}








