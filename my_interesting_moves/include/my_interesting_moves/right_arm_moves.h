// right_arm_moves.h header file //
/// hudy7; Oct, 2015.  
/// Include this file in "right_arm_moves.cpp", and in any main that uses this library.
///This class provides moves to be made with Baxter's right arm
///

#ifndef RIGHT_ARM_MOVES_H_		//avoids header duplication
#define RIGHT_ARM_MOVES_H_

#include <ros/ros.h>
#include <ros/ros.h> 
#include <ros/init.h>
#include <std_msgs/Float32.h> 
#include <baxter_core_msgs/JointCommand.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <cwru_srv/simple_bool_service_message.h>


typedef Eigen::Matrix<double, 6, 1> Vectorq6x1;
typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;
using namespace std;
const double q0dotmax = 0.5;
const double q1dotmax = 0.5;
const double q2dotmax = 0.5;
const double q3dotmax = 0.5;
const double q4dotmax = 1;
const double q5dotmax = 1;
const double q6dotmax = 1;

const double dt_traj = 0.01; // time step for trajectory interpolation

const double SPEED_SCALE_FACTOR= 0.5; // go this fraction of speed from above maxes


class RightArmMoves {
	
	public: 
		RightArmMoves(ros::NodeHandle* nodehandle);

		void cmd_pose_right(Vectorq7x1 qvec);

		void stuff_trajectory( std::vector<Vectorq7x1> qvecs, trajectory_msgs::JointTrajectory &new_trajectory);
    	
    	void stuff_trajectory( std::vector<Eigen::VectorXd> qvecs, trajectory_msgs::JointTrajectory &new_trajectory); 

    	void pub_right_arm_trajectory(trajectory_msgs::JointTrajectory &new_trajectory);
    	
    	Vectorq7x1 get_qvec_right_arm();  
    	
    	void pub_right_arm_trajectory_init();
    	
    	sensor_msgs::JointState get_joint_states() { return joint_states_;}

	private:
		//member variables

		ros::NodeHandle nh_;

		ros::Subscriber joint_state_sub_;

		ros::ServiceClient traj_interp_stat_client_;

		ros::Publisher joint_cmd_pub_right_;
		ros::Publisher joint_cmd_pub_left_;

		ros::Publisher right_traj_pub_;

		double val_from_subscriber;
		double val_to_remember;

		Vectorq7x1 vector_right_arm;
		Vectorq7x1 constraint;

		sensor_msgs::JointState joint_states_;
		baxter_core_msgs::JointCommand right_cmd;
		baxter_core_msgs::JointCommand left_cmd;

		cwru_srv::simple_bool_service_message traj_status;

		//member methods:
		void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
	    void initializePublishers();
	    void initializeServices();
	    void jointStatesCb(const sensor_msgs::JointState& js_msg); //prototype for callback of joint-state messages
	    void map_right_arm_joint_indices(vector<string> joint_names);

	    double transition_time(Vectorq7x1 dqvec_time);
	    double transition_time(Eigen::Vectorq7x1 dqvec_time);
};

#endif

