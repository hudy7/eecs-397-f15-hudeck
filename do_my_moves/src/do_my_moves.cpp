//  Copyright Nicholas Hudeck

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <my_moves/my_moves.h>
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "traj_action_client_node");

  ros::NodeHandle* nh;

  My_moves m;

  m.goToPose(m.raiseHand());

  ros::Duration(2.5);

  m.goToPose(m.wave());

  ros::Duration(1.0);

  m.goToPose(m.slapBeerOffTable());
}
