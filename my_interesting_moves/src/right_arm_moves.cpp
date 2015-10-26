//Author: Nicholas Hudeck
//Date: Oct 25 2015

#include <right_arm_moves/right_arm_moves.h>



//for the purpose of simplicity ra stands for right arm

vector<string> joint_names;
vector<int> ra_joint_indices;


My_interesting_moves::Right_arm_moves(ros::NodeHandle* nodehandle){

  initializeSubscribers();
  initializePublishers();

  right_cmd.mode = 1;

  //definition of joint angles for all 6 joints from shoulder to wrist
  right_cmd.names.push_back("right_s0");
  right_cmd.names.push_back("right_s1");
  right_cmd.names.push_back("right_e0");
  right_cmd.names.push_back("right_e1");
  right_cmd.names.push_back("right_w0");
  right_cmd.names.push_back("right_w1");
  right_cmd.names.push_back("right_w2");


  //establishing desired vector size with valid joint angles
  for(int i = 0; i < 7; i++){
     right_cmd.command.push_back(0.0);		  	
  }


  constraint<<q0dotmax,q1dotmax,q2dotmax,q3dotmax,q4dotmax,q5dotmax,q6dotmax;
  constraint *= SPEED_SCALE_FACTOR; //not sure what this part does

  traj_interp_stat_client_ = nh_.serviceClient<cwru_srv::simple_bool_service_message>("trajInterpStatusSvc");

}

void My_interesting_moves::initializeSubscribers(){
	ROS_INFO("Initialization of Subscribers:");
	joint_state_pub = nh_.serviceClient("robot/joint_states", 1, &My_interesting_moves::jointStatesCb,this);
}

void My_interesting_moves::initializePublishers(){
	ROS_INFO("Initialization of Publishers:");
	joint_cmd_pub_right_ = nh_.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1, true); 
	right_traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("right_arm_joint_path_command", 1); 
}

void My_interesting_moves::map_right_arm_joint_indices(vector<string> joint_names){
	vector<string> rt_limb_jnt_names;

	ra_joint_indices.clear();

	int index;
	int n_jnts = joint_names.size();

	//establish joint names

	cout<<"num jnt names = "<<n_jnts<<endl;
    std::string j_name;

    std::string j_s0_name ("right_s0");
    rt_limb_jnt_names.push_back(j_s0_name);
    
    std::string j_s1_name ("right_s1");
    rt_limb_jnt_names.push_back(j_s1_name);    
    
    std::string j_e0_name ("right_e0");
    rt_limb_jnt_names.push_back(j_e0_name);    
    
    std::string j_e1_name ("right_e1");
    rt_limb_jnt_names.push_back(j_e1_name);    
    
    std::string j_w0_name ("right_w0");
    rt_limb_jnt_names.push_back(j_w0_name);    
    
    std::string j_w1_name ("right_w1");   
    rt_limb_jnt_names.push_back(j_w1_name);    
    
    std::string j_w2_name ("right_w2");  
    rt_limb_jnt_names.push_back(j_w2_name); 

    //looping through joints and performing pushback on all indeces

    for(int j = 0; j<7; j++){
    	j_name = rt_limb_jnt_names[j];
    	for(int i = 0; i < n_jnts; i++){
    		index = i;
    		ra_joint_indices.push_back(index);
    		break;
    	}
    }

    //print out the right arm joints
    cout<<"indices of right-arm joints: "<<endl;
   	for (int i=0;i<7;i++) {
       cout<<ra_joint_indices[i]<<", ";
   	}
   	cout<<endl;

}



void My_interesting_moves::stuff_trajectory(std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory) {
    //new_trajectory.clear();
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    //trajectory_msgs::JointTrajectoryPoint trajectory_point2; 
    
    trajectory_point1.positions.clear(); 
    

    new_trajectory.points.clear(); // can clear components, but not entire trajectory_msgs
    new_trajectory.joint_names.clear(); 
    
    new_trajectory.joint_names.push_back("right_s0");
    new_trajectory.joint_names.push_back("right_s1");
    new_trajectory.joint_names.push_back("right_e0");
    new_trajectory.joint_names.push_back("right_e1");
    new_trajectory.joint_names.push_back("right_w0");
    new_trajectory.joint_names.push_back("right_w1");
    new_trajectory.joint_names.push_back("right_w2");

    new_trajectory.header.stamp = ros::Time::now(); 

    Eigen::VectorXd q_start,q_end,dqvec;
    
    double del_time;
    double net_time=0.0;
    
    q_start = qvecs[0];
    q_end = qvecs[0];   
    
    cout<<"stuff_traj: start pt = "<<q_start.transpose()<<endl; 

 
    trajectory_point1.time_from_start = ros::Duration(net_time); 
    
    for (int i=0;i<7;i++) { //pre-sizes positions vector, so can access w/ indices later
        trajectory_point1.positions.push_back(q_start[i]);
    } 
    
    new_trajectory.points.push_back(trajectory_point1); // first point of the trajectory
    

    //add the rest of the points from qvecs
    for (int iq=1;iq<qvecs.size();iq++) {
        q_start = q_end;
        q_end = qvecs[iq];
        dqvec = q_end-q_start;
        cout<<"dqvec: "<<dqvec.transpose()<<endl;
        del_time = transition_time(dqvec);
        if (del_time< dt_traj)
            del_time = dt_traj;
        cout<<"stuff_traj: next pt = "<<q_end.transpose()<<endl; 
        net_time+= del_time;
        ROS_INFO("iq = %d; del_time = %f; net time = %f",iq,del_time,net_time);        
        for (int i=0;i<7;i++) { //copy over the joint-command values
            trajectory_point1.positions[i]=q_end[i];
        }   
       
        trajectory_point1.time_from_start = ros::Duration(net_time); 
        new_trajectory.points.push_back(trajectory_point1);        
    }        
        
}  


void My_interesting_moves::cmd_pose_right(Vector7x1 qvec){

	for(int i = 0; i<7; i++){
		right_cmd.command[i] = qvec[i];
	}

	joint_cmd_pub_right_.publish(right_cmd);
}

void My_interesting_moves::pub_right_arm_trajectory_init() {
    std::vector<Vectorq7x1> qvecs;
    
    trajectory_msgs::JointTrajectory new_trajectory;
    
    Vectorq7x1 q_snapshot = q_vec_right_arm_;
    
    qvecs.push_back(q_snapshot);
    qvecs.push_back(q_snapshot);    
    stuff_trajectory(qvecs, new_trajectory);   
    
    bool working_on_traj=true;
    
    while (working_on_traj) {
      	traj_interp_stat_client_.call(traj_status_srv_); // communicate w/ trajectory interpolator node status service
        working_on_traj = traj_status_srv_.response.resp;
        cout<<"waiting for ready from interp node..."<<endl;
        ros::spinOnce();
    }
    cout<<"interp node is ready for traj; sending"<<endl;
    working_on_traj=true;

 
    right_traj_pub_.publish(new_trajectory);
    cout<<"publishing trajectory with npts = "<<new_trajectory.points.size()<<endl;
    cout<<"cmd: "<<q_snapshot.transpose()<<endl;
    ros::Duration(0.5).sleep(); // sleep for half a second
    while(working_on_traj) {
      traj_interp_stat_client_.call(traj_status_srv_); // communicate w/ trajectory interpolator node status service
        working_on_traj = traj_status_srv_.response.resp;
        cout<<"waiting for interp node to finish trajectory..."<<endl;
        ros::spinOnce();       
    }   
}












































