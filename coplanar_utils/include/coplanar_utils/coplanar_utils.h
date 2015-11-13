// cwru_pcl_utils.h header file; doxygen comments follow //
/// wsn; Oct, 2015.  
/// Include this file in "my_pcl_utils.cpp", and in any main that uses this library.
///This class provides example functions using the Point Cloud Library to operate
/// on point-cloud data

#ifndef COPLANAR_UTILS_H_
#define COPLANAR_UTILS_H_

#include <ros/ros.h> //generic C++ stuff
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <vector>

#include <Eigen/Eigen> //for the Eigen library
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <sensor_msgs/PointCloud2.h> //useful ROS message types
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <cwru_msgs/PatchParams.h>

#include <tf/transform_listener.h>  // transform listener headers
#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>  //point-cloud library headers; likely don't need all these
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>

using namespace std;  //just to avoid requiring std::, Eigen:: ...
using namespace Eigen;
using namespace pcl;
using namespace pcl::io;

// define a class, including a constructor, member variables and member functions
class CoplanarUtils
{
public:
    CoplanarUtils(ros::NodeHandle* nodehandle); //constructor


    void fit_points_to_plane(Eigen::MatrixXf points_array, 
    Eigen::Vector3f &plane_normal, 
    double &plane_dist); 
    Eigen::Vector3f compute_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr);
    
    void fit_points_to_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,Eigen::Vector3f &plane_normal, double &plane_dist);
    void fit_xformed_selected_pts_to_plane(Eigen::Vector3f &plane_normal, double &plane_dist);  

    
    Eigen::Affine3f transformTFToEigen(const tf::Transform &t);
    void transform_kinect_cloud(Eigen::Affine3f A);
    void transform_selected_points_cloud(Eigen::Affine3f A);
    void transform_cloud(Eigen::Affine3f A,pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr);    
    void reset_got_kinect_cloud() {got_kinect_cloud_= false;};
    void reset_got_selected_points() {got_selected_points_= false;};    
    bool got_kinect_cloud() { return got_kinect_cloud_; };
    bool got_selected_points() {return got_selected_points_;};
    void save_kinect_snapshot() {    pcl::io::savePCDFileASCII ("kinect_snapshot.pcd", *pclKinect_ptr_);};
  
    void save_kinect_clr_snapshot() {pcl::io::savePCDFileASCII ("kinect_clr_snapshot.pcd", *pclKinect_clr_ptr_);};
    void save_transformed_kinect_snapshot() { pcl::io::savePCDFileASCII ("xformed_kinect_snapshot.pcd", *pclTransformed_ptr_);};
    void get_transformed_selected_points(pcl::PointCloud<pcl::PointXYZ> & outputCloud );
    void copy_cloud(PointCloud<pcl::PointXYZ>::Ptr inputCloud, PointCloud<pcl::PointXYZ>::Ptr outputCloud); 
    void get_gen_purpose_cloud(pcl::PointCloud<pcl::PointXYZ> & outputCloud );    
    void example_pcl_operation();
    
    //new function added by Nick Hudeck
    void find_coplanar();
    
private:
    ros::NodeHandle nh_; 
    ros::Subscriber pointcloud_subscriber_;
    ros::Subscriber selected_points_subscriber_; 
    
   
    ros::Publisher  pointcloud_publisher_;
    ros::Publisher patch_publisher_;    
    
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pclKinect_clr_ptr_; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclKinect_ptr_; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclTransformed_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclSelectedPoints_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclTransformedSelectedPoints_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclGenPurposeCloud_ptr_;
    
    
    bool got_kinect_cloud_;
    bool got_selected_points_;
    void initializeSubscribers(); 
    void initializePublishers();
   
    void kinectCB(const sensor_msgs::PointCloud2ConstPtr& cloud);
    void selectCB(const sensor_msgs::PointCloud2ConstPtr& cloud);     
    

}; 
#endif
