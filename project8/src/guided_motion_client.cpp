// Copyright Nicholas Hudeck, Modern Robotics EECS 397

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <cwru_action/cwru_baxter_cart_moveAction.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cwru_pcl_utils/cwru_pcl_utils.h>
#include "arm_motion_commander_class.cpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "guided_motion_client");
  ros::NodeHandle nh;

  /**
  Instantiate an arm motion commander and pcl_utils
  Main relies heavily on methods from these two
  */
  ArmMotionCommander arm_motion_commander(&nh);
  CwruPclUtils cwru_pcl_utils(&nh);

  /**
  Check to make sure that we have received a point cloud
  */
  while (!cwru_pcl_utils.got_kinect_cloud())
  {
    ROS_INFO("did not receive pointcloud");
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  ROS_INFO("got a pointcloud");
  tf::StampedTransform tf_sensor_frame_to_torso_frame;  // use this to transform sensor frame to torso frame
  tf::TransformListener tf_listener;  // start a transform listener

    //  let's warm up the tf_listener, to make sure it get's all the transforms it needs to avoid crashing:
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    while (tferr)
    {
      tferr = false;
      /**
	  Try/Catch block needed while waiting for transfrom between KINECT and PCFrame works with no errors
      */
      try
      {
        tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
      }
      catch (tf::TransformException &exception)
      {
        ROS_ERROR("%s", exception.what());
        tferr = true;
        ros::Duration(0.5).sleep();  // sleep for half a second
        ros::spinOnce();
      }
    }
    ROS_INFO("tf is good");
    // convert the tf to an Eigen::Affine:
    Eigen::Affine3f A_sensor_wrt_torso;
    Eigen::Affine3d Affine_des_gripper;
    Eigen::Vector3d xvec_des, yvec_des, zvec_des, origin_des;
    geometry_msgs::PoseStamped rt_tool_pose;
    A_sensor_wrt_torso = cwru_pcl_utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);
    Eigen::Vector3f plane_normal, major_axis, centroid;
    Eigen::Matrix3d Rmat;
    int rtn_val;
    double plane_dist;

    /**
	Set up the prepose
    */
    rtn_val = arm_motion_commander.plan_move_to_pre_pose();

    /**
    send command to execute planned motion
    */
    rtn_val = arm_motion_commander.rt_arm_execute_planned_path();

    while (ros::ok())
    {
      if (cwru_pcl_utils.got_selected_points())
      {
        ROS_INFO("transforming selected points");
        cwru_pcl_utils.transform_selected_points_cloud(A_sensor_wrt_torso);

        // fit a plane to these selected points:
        cwru_pcl_utils.fit_xformed_selected_pts_to_plane(plane_normal, plane_dist);
        ROS_INFO_STREAM(" normal: " << plane_normal.transpose() << "; dist = " << plane_dist);
        major_axis = cwru_pcl_utils.get_major_axis();
        centroid = cwru_pcl_utils.get_centroid();
        cwru_pcl_utils.reset_got_selected_points();   // reset the selected-points trigger
        // construct a goal affine pose:
        for (int i = 0; i < 3; i++)
        {
          origin_des[i] = centroid[i];  // convert to double precision
          zvec_des[i] = -plane_normal[i];  // want tool z pointing OPPOSITE surface normal
          xvec_des[i] = major_axis[i];
        }
        origin_des[2] += 0.02;  // raise up 2cm
        yvec_des = zvec_des.cross(xvec_des);  // construct consistent right-hand triad
        Rmat.col(0) = xvec_des;
        Rmat.col(1) = yvec_des;
        Rmat.col(2) = zvec_des;

        Affine_des_gripper.linear() = Rmat;
        Affine_des_gripper.translation() = origin_des;

        cout << "des origin: " << Affine_des_gripper.translation().transpose() << endl;
        cout << "orientation: " << endl;
        cout << Rmat << endl;

        // convert des pose from Eigen::Affine to geometry_msgs::PoseStamped
        rt_tool_pose.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
        // could fill out the header as well...

        // send move plan request:
        rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_pose);
        if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)
        {
          // send command to execute planned motion
          rtn_val = arm_motion_commander.rt_arm_execute_planned_path();


          /**
		  Dummie int's for coordinating Baxters arm movement
          */
          int value1 = 0;
          int value2 = 0;
          for (origin_des[1]; value1 < 6; origin_des[1] += 0.1)
          {
            ROS_INFO("Movement to L");
        	geometry_msgs::PoseStamped rt_tool_swipe;
        	Affine_des_gripper.translation() = origin_des;
		  	rt_tool_swipe.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
			rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_swipe);
			value1++;

			if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS)
		    {
			  rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
			}
			else
			{
			  ROS_WARN("CARTESTIAN PATH IS NOT ACHIEVABLE");
			}
			/**
			This may be a hack way of doing this, but basically the "tool" moves backward and forth
			until it hits a said number and returns the other direction

			Its not the smoothest path but it could definitely use some improvement.

			*/
			if (value1 == 2){
			  for (origin_des[1]; value2 < 6; origin_des[1] -= 0.4){
				geometry_msgs::PoseStamped rt_tool_swipe;
				Affine_des_gripper.translation() = origin_des;
				rt_tool_swipe.pose = arm_motion_commander.transformEigenAffine3dToPose(Affine_des_gripper);
				rtn_val = arm_motion_commander.rt_arm_plan_path_current_to_goal_pose(rt_tool_swipe);
				
				value2++;
				
				if (rtn_val == cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
				  rtn_val = arm_motion_commander.rt_arm_execute_planned_path();
				}	 
				
				else {
				  ROS_WARN("CARTESTIAN PATH IS NOT ACHIEVABLE");
				}
			}
		  }
		}
	   }
            
            else 
            {
                ROS_WARN("CARTESTIAN PATH IS NOT ACHIEVABLE");
            }
		}

   
        ros::Duration(0.5).sleep();  // sleep for half a second
        ros::spinOnce();
    }
 

    return 0;
}