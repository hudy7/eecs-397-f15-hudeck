//@Author: Nicholas Hudeck
//Project 8, CWRU Robotics - Modern Robotics

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cwru_action/cwru_baxter_cart_moveAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cwru_pcl_utils/cwru_pcl_utils.h>

#include "arm_motion_commander_class.cpp"
//define a class to encapsulate some of the tedium of populating and sending goals,
// and interpreting responses;


int main(int argc, char** argv) {
    ros::init(argc, argv, "example_cart_move_action_client"); // name this node
    ros::NodeHandle nh; //standard ros node handle
    ArmMotionCommander arm_motion_commander(&nh);
    CwruPclUtils cwru_pcl_utils(&nh);
    while (!cwru_pcl_utils.got_kinect_cloud()) {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }
    ROS_INFO("got a pointcloud");
    tf::StampedTransform tf_sensor_frame_to_torso_frame; //tf sensor frame to torso frame
    tf::TransformListener tf_listener; //activate a transform listener

    //start tf_listener
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    while (tferr) {
        tferr = false;
        try {
            tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); 
    Eigen::Affine3f A_sensor_wrt_torso;
    Eigen::Affine3d Affine_des_gripper;
    Eigen::Vector3d xvec_des,yvec_des,zvec_des,origin_des;
    geometry_msgs::PoseStamped rt_tool_pose;

    A_sensor_wrt_torso = cwru_pcl_utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);
    Eigen::Vector3f plane_normal, major_axis, centroid, temp_goal;
    Eigen::Matrix3d rm;
    int rtn_val;
    double plane_dist;

    //send a command to plan a joint-space move to pre-defined pose:
    rtn_val=arm_motion_commander.plan_move_to_pre_pose();

    //send command to execute planned motion
    rtn_val=arm_motion_commander.rt_arm_execute_planned_path();

    while (ros::ok()) {
        if (cwru_pcl_utils.got_selected_points()) {
            ROS_INFO("transforming selected points");
            cwru_pcl_utils.transform_selected_points_cloud(A_sensor_wrt_torso);
            cwru_pcl_utils.fit_xformed_selected_pts_to_plane(plane_normal, plane_dist);
            ROS_INFO_STREAM(" normal: " << plane_normal.transpose() << "; dist = " << plane_dist);
            major_axis= cwru_pcl_utils.get_major_axis();
            centroid = cwru_pcl_utils.get_centroid();
            cwru_pcl_utils.reset_got_selected_points();   // reset the selected-points trigger

            //create a goal pose:
            for (int i=0;i<3;i++) {
                origin_des[i] = centroid[i]; // convert to double
                zvec_des[i] = -plane_normal[i]; //I want tool z pointing OPPOSITE surface normal (downward on surface)
                xvec_des[i] = major_axis[i];
            }
            origin_des[2]+=0.03; //raise up 3cm from surface
            yvec_des = zvec_des.cross(xvec_des); 



            rm.col(0)= xvec_des;
            rm.col(1)= yvec_des;
            rm.col(2)= zvec_des;
            Affine_des_gripper.linear()=rm;
            Affine_des_gripper.translation()=origin_des;
            cout<<"destination origin:   "<<Affine_des_gripper.translation().transpose()<<endl;
            cout<<"current orientation:   "<<endl;
            cout<<rm<<endl;
            //convert des pose from Eigen::Affine to geometry_msgs::PoseStamped
            rt_tool_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
            

            // sending the move plan request to arm commander:
            rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);

            if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)  {
                ROS_INFO("EXECUTING PLANNED MOVE");
                rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
            }
            else {
                ROS_WARN("POSE IS NOT ACHIEVABLE");
            }

            double dx = 0.01;
            double dy = 0.01;


            //mostly copied from example_baxter_cart_move_client.cpp in class repo
            //Authors: wsn , lab
            for(int iy = 0; iy < 10; iy++) {

            	origin_des[0] = origin_des[0]+0.1;
            	Affine_des_gripper.translation() = origin_des;
	           rt_tool_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
	           rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
	           rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
	           if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)  {
	               rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
	           }
	           else {
	               ROS_WARN("POSE IS NOT ACHIEVABLE");
	           }
            	origin_des[0] = origin_des[0]-0.1;
            	Affine_des_gripper.translation() = origin_des;
	           rt_tool_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
              ROS_INFO("SENDING MOVE PLAN REQUEST");
	           rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
            	// send move plan request:
	           rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
	           //if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)  {
	               //send command to execute planned motion
	               rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
	           /*}
	           else {
	               ROS_WARN("Cartesian path to desired pose not achievable");
				}*/

            	origin_des[1] = origin_des[1]+iy*dy;
            	Affine_des_gripper.translation() = origin_des;
	           rt_tool_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);

	           // send move plan request:
	           rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
            	// send move plan request:
	           rtn_val=arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);

	           if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)  {
	               rtn_val=arm_motion_commander.rt_arm_execute_planned_path();
	           }
	           else {
	               ROS_WARN("POSE NOT ACHIEVABLE");
	           }

            }
        }
        ros::Duration(0.5).sleep(); 
        ros::spinOnce();
    }

    return 0;
}