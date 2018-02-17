#include "dynamic_window.h"


Dynamic_window::Dynamic_window(MapParam* _pMapParam)
{
	pMapParam=_pMapParam;
	Init();
}

void Dynamic_window::setPMapParam(MapParam* _pMapParam)
{

	pMapParam=_pMapParam;
	Init();
}

Dynamic_window::~Dynamic_window()
{
	if(pMapParam!=NULL)
	{
		delete pMapParam;
		pMapParam=NULL;
	}
}



void Dynamic_window::Init()
{
	cout<<"Initialize"<<endl;
	Local_X_start=0;
 	Local_Y_start=0;
 	
 	X_mapSize=pMapParam->Num_grid_X;
 	Y_mapSize=pMapParam->Num_grid_Y;
 	Num_Grids=pMapParam->MapSize;
 	cout<<" Grid - X :"<<X_mapSize<<", - Y : "<<Y_mapSize<<endl;
 	cout<<" NumofGrids : "<<Num_Grids<<endl;

	 Map_orig_Vector.resize(2,0.0);
	 Map_orig_Vector[0]= 3.5;
   	 Map_orig_Vector[1]=-3.5;

   	 global_pose.resize(3,0.0);
     CurVector.resize(3,0.0);
	 m_dynamic_occupancy.resize(Num_Grids);

	 Head_Pos.resize(2,0.0);

	//Declare publisher
	Scaled_static_map_pub=m_node.advertise<nav_msgs::OccupancyGrid>("/scaled_static_map", 10, true);
	Scaled_dynamic_map_pub=m_node.advertise<nav_msgs::OccupancyGrid>("/scaled_dynamic_map", 10, true);
	Scaled_dynamic_map_path_pub=m_node.advertise<nav_msgs::OccupancyGrid>("/scaled_dynamic_map_path", 10, true);
	camera_map_pub=m_node.advertise<nav_msgs::OccupancyGrid>("/camera_region_map", 10, true);

	Scaled_static_map_path.info.width=Grid_Num_X;
	Scaled_static_map_path.info.height= Grid_Num_Y;
	Scaled_static_map_path.info.resolution=0.5;
	Scaled_static_map_path.info.origin.position.x=-4;
	Scaled_static_map_path.info.origin.position.y=-4;
	Scaled_static_map_path.data.resize(Scaled_static_map_path.info.width*Scaled_static_map_path.info.height);

   	Scaled_dynamic_map_path.info.width=10;
	Scaled_dynamic_map_path.info.height= 10;
	Scaled_dynamic_map_path.info.resolution=0.75;
	Scaled_dynamic_map_path.info.origin.position.x=CurVector[0]-DYN_OFFSET_X-0.5*Scaled_dynamic_map.info.resolution;
	Scaled_dynamic_map_path.info.origin.position.y=CurVector[1]-DYN_OFFSET_Y-0.5*Scaled_dynamic_map.info.resolution;;
	Scaled_dynamic_map_path.data.resize(Scaled_dynamic_map_path.info.width*Scaled_dynamic_map_path.info.height);
	dyn_path_num=0;

  //camera region
  camera_map.info.width=30;
  camera_map.info.height= 30;
  camera_map.info.resolution=0.5;
  camera_map.info.origin.position.x=-7.5;
  camera_map.info.origin.position.y=-7.5;
  int camera_map_size=camera_map.info.width*camera_map.info.height;
  camera_map.data.resize(camera_map_size);
  for(int k(0);k<camera_map_size;k++)
    camera_map.data[k]=0.01;

}


void Dynamic_window::joint_states_callback(const sensor_msgs::JointState::ConstPtr& msg)
{

  Head_Pos[0]=msg->position[9];     //pan
  Head_Pos[1]=msg->position[10];      //tilt

}

bool Dynamic_window::IsinDynamicMap(float global_x, float global_y)
{
	float margin =0.1;
	float map_start_x=Scaled_dynamic_map.info.origin.position.x-margin;
	float map_start_y=Scaled_dynamic_map.info.origin.position.y-margin;
	float map_end_x =Scaled_dynamic_map.info.origin.position.x+Scaled_dynamic_map.info.width*Scaled_dynamic_map.info.resolution+margin;
	float map_end_y =Scaled_dynamic_map.info.origin.position.y+Scaled_dynamic_map.info.height*Scaled_dynamic_map.info.resolution+margin;

	if( (global_x> map_start_x) && (global_x< map_end_x) )
		if((global_y> map_start_y) && (global_y< map_end_y) )
			return true;

	return false;
}


double Dynamic_window::getdistance(vector<double> cur, vector<double> goal)
{
	double temp_dist=0.0;
	for(int i(0);i<2;i++)
		temp_dist+=pow((cur[i]-goal[i]),2);
	
	temp_dist = sqrt(temp_dist);

	return temp_dist;

}



void Dynamic_window::mdppath_callback(const nav_msgs::Path::ConstPtr & msg)
{
    //nav_msgs::Path mdp_path_msg;    
    //mdp_path_msg=(*msg);
    //std::vector<int> dyn_coords(2,0);

    //Dyn_MDPPath.clear();
    //for(int j(0);j<Scaled_dynamic_map_path.data.size();j++)
	 //{	
		 //Scaled_dynamic_map_path.data[j]=0.0;
	 //}


	//int path_size=mdp_path_msg.poses.size();
    //for(int i(0);i<path_size;i++)
    //{
         //std::cout<<mdp_path_msg.poses[i].pose.position.x<<", "<<mdp_path_msg.poses[i].pose.position.y<<std::endl;

        //CoordinateTansform_Rviz_Dyn_map(mdp_path_msg.poses[i].pose.position.x,mdp_path_msg.poses[i].pose.position.y,dyn_coords);       
		//int cur_stid=Coord2CellNum(dyn_coords);
		//Scaled_dynamic_map_path.data[cur_stid]=80;
    //}

	 //Scaled_dynamic_map_path.header.stamp =  ros::Time::now();
	 //Scaled_dynamic_map_path.header.frame_id = "map"; 
     //Scaled_dynamic_map_path_pub.publish(Scaled_dynamic_map_path);
}


void Dynamic_window::Human_MarkerCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
/*
	//float human_target_goal_x=  msg->pose.position.x;
    //float human_target_goal_y=  msg->pose.position.y;

    //CoordinateTransform_Rviz_Grid_Human(human_target_goal_x,human_target_goal_y,1);
   

   check if person moved a lot 
   //if( (dyn_path_num>0) && IsTargetMoved(human_target_goal_x,human_target_goal_y,3.0))
       //return;

   check if person is in a range
    if(!IsinDynamicMap(human_target_goal_x,human_target_goal_y))   
    {
           ROS_INFO("human is out of my dynamic range");
           return;
    }

    dynamic goal setting
    if(booltrackHuman)
    {
        //if(dyn_path_num>0 && (!IsinDynamicMap(human_target_goal_x,human_target_goal_y)))
            //return;

        if(dyn_path_num==0 || IsTargetMoved(human_target_goal_x,human_target_goal_y, 0.5))
        { 
            global coordinate
			//GoalVector.resize(2,0);
			//GoalVector[0]=human_target_goal_x;
            GoalectoFiltered_leg_human.clear();

H   */
}

void Dynamic_window::Human_MarkerArrayCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{

    int num_of_detected_human=msg->markers.size();

    if(num_of_detected_human>0)
       cur_people.resize(num_of_detected_human);
    else
    {
      return;
    }

    for(int i(0);i<num_of_detected_human;i++)
    {
      geometry_msgs::Vector3Stamped gV, tV;

      gV.vector.x = msg->markers[i].pose.position.x;
      gV.vector.y = msg->markers[i].pose.position.y;
      gV.vector.z = msg->markers[i].pose.position.z;

      // std::cout<<"x :"<<_x<<"_y:"<<_y<<"_z:"<<_z<<std::endl;
      tf::StampedTransform maptransform;
      listener.waitForTransform("head_rgbd_sensor_rgb_frame", "map", ros::Time(0), ros::Duration(1.0));
              
      gV.header.stamp = ros::Time();
      gV.header.frame_id = "/head_rgbd_sensor_rgb_frame";
      listener.transformVector(std::string("/map"), gV, tV);
              
      cur_people[i].resize(2,0.0);
      cur_people[i][0]=tV.vector.x+global_pose[0];
      cur_people[i][1]=tV.vector.y+global_pose[1];
   }
 
    //CoordinateTransform_Rviz_Grid_Human(human_target_goal_x,human_target_goal_y,1);
 

}
bool Dynamic_window::Comparetwopoistions(std::vector<double> pos,std::vector<double> pos2, double criterion)
{

  double temp_dist=getdistance(pos,pos2);

  if(temp_dist<criterion)
    return true;
  
  return false;

}

//
void Dynamic_window::CellNum2globalCoord(const int Cell_idx, std::vector<double>& cell_xy)
{
	  cell_xy.resize(2,0.0);

	  int res =(int) Cell_idx / Human_Belief_Scan_map.info.width;
	  int div =(int) Cell_idx % Human_Belief_Scan_map.info.width;

	  cell_xy[0]=Human_Belief_Scan_map.info.resolution*div+0.5*Human_Belief_Scan_map.info.resolution+Human_Belief_Scan_map.info.origin.position.x;
	  cell_xy[1]=Human_Belief_Scan_map.info.resolution*res+0.5*Human_Belief_Scan_map.info.resolution+Human_Belief_Scan_map.info.origin.position.y;
}

void Dynamic_window::ClikedpointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	printf("Dynamic goal Receive point\n");
	 
    return;
}


bool Dynamic_window::NotUpdatedCameraregion(int idx)
{
	for(int i(0);i<visiblie_idx_set.size();i++)
	{
		if(idx==visiblie_idx_set[i])
			return false;
	}

	return true;
}


void Dynamic_window::dynamic_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
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

	//for dynamic map space map
	Scaled_dynamic_map.info.width=10;
	Scaled_dynamic_map.info.height= 10;
	Scaled_dynamic_map.info.resolution=0.75;
	Scaled_dynamic_map.info.origin.position.x=CurVector[0]-DYN_OFFSET_X-0.5*Scaled_dynamic_map.info.resolution;
	Scaled_dynamic_map.info.origin.position.y=CurVector[1]-DYN_OFFSET_Y-0.5*Scaled_dynamic_map.info.resolution;
	// Scaled_dynamic_map.info.origin.position.x=CurVector[0]-0.5*Scaled_dynamic_map.info.width*0.5;
	// Scaled_dynamic_map.info.origin.position.y=CurVector[1]-0.5*Scaled_dynamic_map.info.height*0.5;
	Scaled_dynamic_map.data.resize(Scaled_dynamic_map.info.width*Scaled_dynamic_map.info.height);


	Scaled_dynamic_map_path.info.width=10;
	Scaled_dynamic_map_path.info.height= 10;
	Scaled_dynamic_map_path.info.resolution=0.75;
	Scaled_dynamic_map_path.info.origin.position.x=CurVector[0]-DYN_OFFSET_X-0.5*Scaled_dynamic_map.info.resolution;
	Scaled_dynamic_map_path.info.origin.position.y=CurVector[1]-DYN_OFFSET_Y-0.5*Scaled_dynamic_map.info.resolution;

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

   			if(numcount>35)
   				scaled_result=50;
   			else
   				scaled_result=0;

   			Scaled_dynamic_map.data[map_idx]=scaled_result;
   	}

   //insert mdp_path_cell
   //find index from
   Scaled_dynamic_map.header.stamp =  ros::Time::now();
   Scaled_dynamic_map.header.frame_id = "map"; 
   Scaled_dynamic_map_pub.publish(Scaled_dynamic_map);

   for(int i(0);i<m_dynamic_occupancy.size();i++)
       m_dynamic_occupancy[i]=Scaled_dynamic_map.data[i];
}


void Dynamic_window::static_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	// ROS_INFO("staticmap callback start");

	double small_pos_x, small_pos_y=0.0;
	double dist_x,dist_y=0.0;
	int map_coord_i,map_coord_j=0;
	int numcount=0;
	int	original_width=msg->info.width;
	int	original_height= msg->info.height;
	double original_x=-51.225;
	double original_y=-51.225;;
	double oroginal_res=0.05;

	//for static space map
	Scaled_static_map.info.width=Grid_Num_X;
	Scaled_static_map.info.height= Grid_Num_Y;
	Scaled_static_map.info.resolution=0.5;
	Scaled_static_map.info.origin.position.x=-4;
	Scaled_static_map.info.origin.position.y=-4;
	Scaled_static_map.data.resize(Scaled_static_map.info.width*Scaled_static_map.info.height);

	// if(msg->data[0]!=NULL)
	// {
		int datasize=msg->data.size();

   double base_origin_x =msg->info.origin.position.x;
   double base_origin_y =msg->info.origin.position.y;

	std::map<int,int> occupancyCountMap;
    int scaled_res=10;
    int map_idx=0;
    int scaled_result=0;

   for(int j(0);j<Scaled_static_map.info.height;j++)
   	for(int i(0);i<Scaled_static_map.info.width;i++)
   	{
   		map_idx=j*Scaled_static_map.info.height+i;
   		double pos_x=i*Scaled_static_map.info.resolution+Scaled_static_map.info.origin.position.x;
   		double pos_y=j*Scaled_static_map.info.resolution+Scaled_static_map.info.origin.position.y;

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
				
				int map_data_index=original_width*map_coord_j+map_coord_i;
				float temp_occupancy= msg->data[map_data_index];
    			 if(temp_occupancy>0)
				 	numcount++;
   			}

   			if(numcount>5)
   				scaled_result=50;
   			else
   				scaled_result=0;

   			Scaled_static_map.data[map_idx]=scaled_result;
   	}

     //find index from
	 Scaled_static_map.header.stamp =  ros::Time::now();
	 Scaled_static_map.header.frame_id = "map"; 
     Scaled_static_map_pub.publish(Scaled_static_map);
}

void Dynamic_window::global_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	global_pose[0]=msg->pose.position.x;
	global_pose[1]=msg->pose.position.y;

   tf::StampedTransform baselinktransform;
   listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(4.0));
   listener.lookupTransform("map", "base_link", ros::Time(0), baselinktransform);
   double yaw_tf =   tf::getYaw(baselinktransform.getRotation()); 

	global_pose[2]=yaw_tf;

   CurVector[0]= global_pose[0];
   CurVector[1]= global_pose[1];
   CurVector[2]= global_pose[2];

}


void Dynamic_window::Global2MapCoord(const vector<double>& _globalcoord,vector<int>& _MapCoord)
{
 
 _MapCoord.resize(2);
    
  _MapCoord[0]= (int)_globalcoord[0]/(pMapParam->map_step);
  _MapCoord[1]= (int)_globalcoord[1]/(pMapParam->map_step);

  //std::cout<<"x:"<<MapCoord[0]<<" , "<<"y:"<<MapCoord[1]<<std::endl;
  return ;
}


vector<int> Dynamic_window::Global2LocalCoord(vector<int> Global_coord)
{
	vector<int> Local_coords(2,0);
	
	Local_coords[0]=Global_coord[0]-Local_X_start;
	Local_coords[1]=Global_coord[1]-Local_Y_start;

	return Local_coords;
}


int  Dynamic_window::Coord2CellNum(std::vector<int> cell_xy)
{
	int index= cell_xy[0]+X_mapSize*cell_xy[1];


	return index;
}

void Dynamic_window::CellNum2Coord(const int Cell_idx, vector<int>& cell_xy)
{
	  cell_xy.resize(2,0);

	  int res =(int) Cell_idx / X_mapSize;
	  int div =(int) Cell_idx % X_mapSize;

	  cell_xy[0]=div;
	  cell_xy[1]=res;
}
void Dynamic_window::Mapcoord2GlobalCoord(const vector<int>& _Mapcoord, vector<double>& GlobalCoord)
{

	GlobalCoord.resize(2);
	//globalCoord origin x, y;
	GlobalCoord[0]=Scaled_static_map.info.origin.position.x+Scaled_static_map.info.resolution*_Mapcoord[0]+0.5*Scaled_static_map.info.resolution;
	GlobalCoord[1]=Scaled_static_map.info.origin.position.y+Scaled_static_map.info.resolution*_Mapcoord[1]+0.5*Scaled_static_map.info.resolution;

	GlobalCoord[0]=Scaled_static_map.info.origin.position.x+Scaled_static_map.info.resolution*_Mapcoord[0]+0.5*Scaled_static_map.info.resolution;
	GlobalCoord[1]=Scaled_static_map.info.origin.position.y+Scaled_static_map.info.resolution*_Mapcoord[1]+0.5*Scaled_static_map.info.resolution;

}

void Dynamic_window::Mapcoord2DynamicCoord(const vector<int>& _Mapcoord, vector<double>& dynamicCoord)
{
	dynamicCoord.resize(2);
	dynamicCoord[0]=Scaled_dynamic_map.info.origin.position.x+Scaled_dynamic_map.info.resolution*_Mapcoord[0]+0.5*Scaled_dynamic_map.info.resolution;
	dynamicCoord[1]=Scaled_dynamic_map.info.origin.position.y+Scaled_dynamic_map.info.resolution*_Mapcoord[1]+0.5*Scaled_dynamic_map.info.resolution;
}


void Dynamic_window::publish_cameraregion()
{
   getCameraregion();
   camera_map.header.stamp =  ros::Time::now();
   camera_map.header.frame_id = "map"; 
   camera_map_pub.publish(camera_map);
}


void Dynamic_window::getCameraregion()
{

  double global_robot_x= global_pose[0];
  double global_robot_y= global_pose[1];
  double global_robot_theta = global_pose[2]+Head_Pos[0];

  visiblie_idx_set.clear();

  global_robot_theta=0.0;
  //Iteration for belief grid
  for(int i(0);i<camera_map.info.width;i++)
    for(int j(0);j<camera_map.info.height;j++)
  {
    int camera_map_idx=j*camera_map.info.height+i;

    double map_ogirin_x = camera_map.info.origin.position.x;
    double map_ogirin_y = camera_map.info.origin.position.y;

    double trans_vector_x=(i+0.5)*camera_map.info.resolution;
    double trans_vector_y=(j+0.5)*camera_map.info.resolution;

    double rot_trans_vector_x = cos(global_robot_theta)*trans_vector_x-sin(global_robot_theta)*trans_vector_y;
    double rot_trans_vector_y = sin(global_robot_theta)*trans_vector_x+cos(global_robot_theta)*trans_vector_y;

    double belief_global_x=map_ogirin_x+rot_trans_vector_x;
    double belief_global_y=map_ogirin_y+rot_trans_vector_y;

    //solve
    bool line1_result =getlinevalue(1,belief_global_x,belief_global_y);
    bool line2_result =getlinevalue(2,belief_global_x,belief_global_y);


    if( line1_result && line2_result )
    {
      camera_map.data[camera_map_idx]=30;  
      visiblie_idx_set.push_back(camera_map_idx);         //save cell_id 
    }
    else
      camera_map.data[camera_map_idx]=0.0; 
  }


}

bool Dynamic_window::check_cameraregion(float x_pos,float y_pos)
{

   double max_boundary = 7.5 ; //(height or width)*resolution *0.5

 if(abs(x_pos)<max_boundary && abs(y_pos)<max_boundary)
  {
  //return true if it is occupied with obstacles
  if (camera_map.data.size()>0)
  {   
    int obs_idx=CoordinateTransform_Global2_cameraMap(x_pos,y_pos);
    
    if(obs_idx<camera_map.data.size()){
      if(camera_map.data[obs_idx]>10.0)
        return true;
      else
        return false;
    }
  }

  }

  return true;
}



int Dynamic_window::CoordinateTransform_Global2_cameraMap(float global_x, float global_y)
{
  double reference_origin_x=camera_map.info.origin.position.x;
  double reference_origin_y=camera_map.info.origin.position.y;

  //Find the coordinate w.r.t map origin
  double  temp_x  = global_x - reference_origin_x;
  double  temp_y  = global_y - reference_origin_y;

  //Find the map cell idx for x, y
  std::vector<int> human_coord(2,0);
  human_coord[0]= (int) (temp_x/camera_map.info.resolution);
  human_coord[1]= (int) (temp_y/camera_map.info.resolution);

  //Find the map index from cell x, y
  int static_map_idx= human_coord[0]+camera_map.info.width*human_coord[1];

  // std::cout<<"map_idx : "<<static_map_idx<<std::endl;
  return static_map_idx;
   
}



bool Dynamic_window::getlinevalue(int line_type,double input_x, double input_y)
{

  double global_robot_theta = global_pose[2]+Head_Pos[0];
  // double global_robot_theta =Camera_angle;
  double theta_1=-FOVW*MATH_PI/180+global_robot_theta;
  double theta_2= FOVW*MATH_PI/180+global_robot_theta;
  
  double m_1=tan(theta_1);
  double m_2=tan(theta_2);

  int isspecial=0;

  if(theta_1<-MATH_PI/2.0 && theta_2 >-MATH_PI/2.0)
  {
    double temp=m_2;
    isspecial=1;
  }
  else if(theta_2> MATH_PI/2.0 && (theta_1 <MATH_PI/2.0))
  {
    isspecial=2;
  }
  else if (theta_1<-MATH_PI/2.0 && theta_2 <-MATH_PI/2.0)
  {
    isspecial=5;
  }
  else if(theta_2< -MATH_PI/2.0)
  {
    isspecial=3;
  }

  else if(theta_1>MATH_PI/2.0 && theta_2> MATH_PI/2.0)
  {
    isspecial=4;  
  }


   // std::cout<<"camera region section : "<<isspecial<<std::endl;
  
  double m=0.0;
  double coeff_sign=1.0;

  double global_robot_x= global_pose[0];
  double global_robot_y= global_pose[1];

  double res =0.0;

  switch(line_type){
  case 1:
      m=m_1;
      coeff_sign=-1.0;

      if(isspecial==0)
          coeff_sign=-1.0;
      else if(isspecial==1)
        coeff_sign=1.0;
      else if(isspecial==2)
        coeff_sign=-1.0;  
      else if(isspecial==4)
        coeff_sign=1.0; 
      else if(isspecial==5)
        coeff_sign=1.0; 

      break;
  case 2:
      m=m_2;
      coeff_sign=-1.0;
      if(isspecial==1)
        coeff_sign=1.0; 
      else if(isspecial==0)
        coeff_sign=1.0; 
      else if(isspecial==3)
        coeff_sign=1.0;
           

      break;
  default:
    std::cout<<"Wrong line type"<<std::endl;
      m=m_1;
    }

  res= m*input_x-m*global_robot_x+global_robot_y-input_y;

  if(res*coeff_sign>0 || res==0)
    return true;
  else
    return false;

}


