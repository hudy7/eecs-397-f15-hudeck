#include <ros/ros.h>
#include <coplanar_utils/coplanar_utils.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "coplanar_points");
    ros::NodeHandle nh;

    // create instance of coplanar_utils library
    CoplanarUtils pcl_utils(&nh);

    
    while (!pcl_utils.got_kinect_cloud())
    {
        ROS_INFO("did not receive pointcloud");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }

    ROS_INFO("got a pointcloud");
    ROS_INFO("saving pointcloud");
    pcl_utils.save_kinect_snapshot();
    pcl_utils.save_kinect_clr_snapshot();

    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/pcl_cloud_display", 1);

    pcl::PointCloud<pcl::PointXYZ> display_cloud;  
    sensor_msgs::PointCloud2 pcl2_display_cloud;   

    tf::StampedTransform tf_sensor_frame_to_torso_frame;  // use this to transform from sensor to torso frame
    tf::TransformListener tf_listener;

    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    while (tferr)
    {
        tferr = false;
        try
        {
            tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
        }
        catch (tf::TransformException &exception)
        {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep();
            ros::spinOnce();
        }
    }

    ROS_INFO("tf is good.");

    //convert the tf to an Eigen::Affine:
    Eigen::Affine3f A_sensor_wrt_torso;
    A_sensor_wrt_torso = pcl_utils.transformTFToEigen(tf_sensor_frame_to_torso_frame);
    pcl_utils.transform_kinect_cloud(A_sensor_wrt_torso);
    pcl_utils.save_transformed_kinect_snapshot();

    Eigen::Vector3f plane_normal;
    double plane_dist;
    while (ros::ok()){
        if (pcl_utils.got_selected_points()){
            pcl_utils.transform_selected_points_cloud(A_sensor_wrt_torso);
            pcl_utils.reset_got_selected_points();
            pcl_utils.fit_xformed_selected_pts_to_plane(plane_normal, plane_dist);
            ROS_INFO_STREAM("Normal: " << plane_normal.transpose() << "Dist: " << plane_dist);

            //this is where we use new function to find coplanar points
            //documentation in header file
            pcl_utils.find_coplanar();
            pcl_utils.get_gen_purpose_cloud(display_cloud);
        }

        
        pcl::toROSMsg(display_cloud, pcl2_display_cloud);
        pcl2_display_cloud.header.stamp = ros::Time::now();
        pcl2_display_cloud.header.frame_id = "torso";

        //this is where we publish the coplanar points
        pubCloud.publish(pcl2_display_cloud);

        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }

    ROS_INFO("COMPLETE!!!");
}
