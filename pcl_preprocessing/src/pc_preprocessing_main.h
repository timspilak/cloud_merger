/**
 * @file pc_preprocessing_main.h
 * @author Tim Spilak (tspilak@stud.hs-heilbronn.de)
 * @brief 
 * @version 0.1
 * @date 2022-04-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */
// general
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

// filter outliers
#include <pcl/filters/radius_outlier_removal.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// filter areas
#include <pcl/filters/passthrough.h>

// remove ground
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Dense>

// voxelgrid
#include <pcl/filters/voxel_grid.h>

// flags
bool flag_tf = false;
bool flag_fused_pc = false;

// true: processed single cloud; false: fused all clouds
bool flag_front_right = false;
bool flag_front_left = false;
bool flag_rear_right = false;
bool flag_rear_left = false;
bool flag_top_middle = false;
bool flag_front_middle = false;

// publisher
ros::Publisher nogroundpub;         // Publisher for no ground Pointcloud (/points_no_ground)
ros::Publisher groundpub;           // Publisher for ground Pointcloud (/points_ground)
ros::Publisher voxelpub;            // Publisher for ground Pointcloud (/points_voxel)

// transformations
tf::StampedTransform front_right_stf;
tf::StampedTransform front_left_stf;
tf::StampedTransform rear_right_stf;
tf::StampedTransform rear_left_stf;
tf::StampedTransform top_middle_stf;
tf::StampedTransform front_middle_stf;

// Pointclouds
pcl::PointCloud<pcl::PointXYZI> front_right_velodyne_no_ground;     
pcl::PointCloud<pcl::PointXYZI> front_right_velodyne_ground;        
pcl::PointCloud<pcl::PointXYZI> front_left_velodyne_no_ground;      
pcl::PointCloud<pcl::PointXYZI> front_left_velodyne_ground;         
pcl::PointCloud<pcl::PointXYZI> rear_right_velodyne_no_ground;      
pcl::PointCloud<pcl::PointXYZI> rear_right_velodyne_ground;         
pcl::PointCloud<pcl::PointXYZI> rear_left_velodyne_no_ground;       
pcl::PointCloud<pcl::PointXYZI> rear_left_velodyne_ground;          
pcl::PointCloud<pcl::PointXYZI> top_middle_velodyne_no_ground;      
pcl::PointCloud<pcl::PointXYZI> top_middle_velodyne_ground;         
pcl::PointCloud<pcl::PointXYZI> livox_no_ground;                    
pcl::PointCloud<pcl::PointXYZI> livox_ground;                       

// functions
void getROI(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr);
void getCloudPart(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_part_ptr, const float length, const float deviation);
void removeGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr, const float z_min_ground, const float z_max_ground, const float max_angle);
void fusePointclouds(pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr);
void voxelgrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud_ptr);
void outlierRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);
void publishPointcloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr, const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr, const pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud_ptr);
void proceedFront(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr);
void proceedRear(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr);


