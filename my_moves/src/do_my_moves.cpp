#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_moves/my_moves.h>
using namespace std;

int main(int argc, char** argv){
	//ros::init(argc,argv,"traj_action_client_node");

	ros::NodeHandle* nh;

	My_moves m;
	//Vectorq7x1 v = m.wave();
	//m.goToPose(v);




}
