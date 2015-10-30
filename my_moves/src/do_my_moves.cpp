#include <ros/ros.h>
#include <my_moves/my_moves.h>
using namespace std;

int main(int argc, char** argv){
	//ros::init(argc,argv,"traj_action_client_node");

	ros::NodeHandle nh;

	MyMoves m(&nh);
	//Vectorq7x1 wavePose = m.wave();
	//m.goToPose(wavePose);




}
