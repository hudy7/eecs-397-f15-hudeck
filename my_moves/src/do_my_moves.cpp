#include <ros/ros.h>
#include <my_moves/my_moves.h>
using namespace std;

int main(int argc, char** argv){

	ros::NodeHandle nh;

	MyMoves(nh) m;
	m.goToPose(wave);




}
