#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_moves/my_moves.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <baxter_traj_streamer/trajAction.h>
using namespace std;

#define VECTOR_DIM 7 //7-dof vector

MyMoves::MyMoves(ros::NodeHandle* nodehandle){};

void doneCb(const actionlib::SimpleClientGoalState& state,
        const baxter_traj_streamer::trajResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got return val = %d; traj_id = %d",result->return_val,result->traj_id);
}


void goToPose(Vectorq7x1 pose){
	ros::NodeHandle nh;

	Baxter_traj_streamer baxter_traj_streamer(&nh);
	Eigen::VectorXd q_in_vecxd;
	Vectorq7x1 q_vec_right_arm;
	std::vector<Eigen::VectorXd> des_path;
	trajectory_msgs::JointTrajectory des_trajectory; //empty trajectory


	cout<<"Warm up call backs";
	for(int i = 0; i<100; i++){
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}

	cout<<"Getting current position of right arm:";
	q_vec_right_arm = baxter_traj_streamer.get_qvec_right_arm();

	cout<<"State of right arm: "<<q_vec_right_arm.transpose();

	q_in_vecxd = q_vec_right_arm;
	des_path.push_back(q_in_vecxd);
	q_in_vecxd = pose;
	des_path.push_back(q_in_vecxd);

	cout<< "Stuffing the trajectory in baxter_traj_streamer:";
	baxter_traj_streamer.stuff_trajectory(des_path, des_trajectory);
	baxter_traj_streamer::trajGoal goal;
	goal.trajectory = des_trajectory;

	//connect to action server
	actionlib::SimpleActionClient<baxter_traj_streamer::trajAction> action_client("trajActionServer",true);

	bool server_exists = action_client.waitForServer(ros::Duration(5.0));

	if(!server_exists){ROS_WARN("COULD NOT CONNECT TO SERVER!"); return;}

	ROS_INFO("CONNECTED TO SERVER");

	action_client.sendGoal(goal, &doneCb);
	return;

}

Vectorq7x1 wave(trajectory_msgs::JointTrajectory &des_trajectory){
	Vectorq7x1 wave;
	wave<<-1.01, .62,   0,    1.993,    3.5,   2,  0;
}


	




Vectorq7x1 flex(trajectory_msgs::JointTrajectory &des_trajectory);
Vectorq7x1 salute(trajectory_msgs::JointTrajectory &des_trajectory);
