///my_moves header file //
///nick hudeck 
///include this file in "my_moves.cpp" and in any main that wishes to use this library
#ifndef MY_MOVES_H
#define MY_MOVES_H

#include <ros/ros.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <baxter_core_msgs/JointCommand.h>
//#include <actionlib/client/simple_action_client.h>
//#include <actionlib/client/terminal_state.h>
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
#include <baxter_traj_streamer/baxter_traj_streamer.h>

typedef Eigen::Matrix<double,7,1> Vectorq7x1;
using namespace std;


/*
const double q0dotmax = 0.5;
const double q1dotmax = 0.5;
const double q2dotmax = 0.5;
const double q3dotmax = 0.5;
const double q4dotmax = 1;
const double q5dotmax = 1;
const double q6dotmax = 1;
const double dt_traj = 0.01; // time step for trajectory interpolation
const double SPEED_SCALE_FACTOR= 0.5; // go this fraction of speed from above maxes
*/

class My_moves{

public:
	/**Constructor
	*/
	My_moves();


	/**Main function of library:
	takes in a pose, performs some necessary arithmetic
	stuffs trajectory with pose values and sends to client!
	*/
	void goToPose(Vectorq7x1 qvec);

	/**
	Returns the necessary joint states to put baxter into wave pose
	*/
	Vectorq7x1 wave();

	/**
	Returns the necessary joint states to put baxter into a raisehand pose
	*/
	Vectorq7x1 raiseHand();

	/**
	Returns the necessary joint states to put baxters right arm on table and pushes beer can off table
	*/
	Vectorq7x1 slapBeerOffTable();

private:
	ros::NodeHandle nh_;

};

#endif
