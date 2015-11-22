# project8

This project shows baxter recognizing a selected number of points and planning a move to go over the selected points in a "wiping" action.. It was also very difficult until I realized most of the code was already in the professors repo! 

## Example usage
* start up and enable the robot (or simulator)

`roslaunch cwru_baxter_sim baxter_world.launch` (or start real robot)

* wait for the robot (or Gazebo) to finish coming up; then enable the robot with:

* start up the action servers and transform publishers with the following commands in separate terminals

`rosrun baxter_tools enable_robot.py -e` 

`rosrun baxter_traj_streamer  traj_interpolator_as`

`rosrun baxter_cartesian_moves baxter_cart_move_as`

`roslaunch cwru_baxter_launch yale_gripper_xform.launch` (to see gripper frame in rviz)

Watch out for the following; kinect transform is different for Gazebo vs real Baxter
`roslaunch cwru_baxter_sim kinect_xform.launch`

* start up rviz and the example sensor-guided motion node
`rosrun rviz rviz` (and set display to see kinect/depth/points and gripper frame)

* this is my client you must run to perform "wiping action"

`rosrun guided_motion_client guided_motion_client`

## Running tests/demos
    
