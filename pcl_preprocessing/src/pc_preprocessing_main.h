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

// General
bool got_transformations = false;
ros::Publisher nogroundpub;         // Publisher for no ground Pointcloud (/points_no_ground)
ros::Publisher groundpub;           // Publisher for ground Pointcloud (/points_ground)
ros::Publisher voxelpub;            // Publisher for ground Pointcloud (/points_voxel)
tf::StampedTransform front_right_stf;
tf::StampedTransform front_left_stf;
tf::StampedTransform rear_right_stf;
tf::StampedTransform rear_left_stf;
tf::StampedTransform top_middle_stf;
tf::StampedTransform front_middle_stf;

// Pointclouds
pcl::PointCloud<pcl::PointXYZI> front_right_velodyne_no_ground;     // Pointcloud to save no ground
pcl::PointCloud<pcl::PointXYZI> front_right_velodyne_ground;        // Pointcloud to save ground

pcl::PointCloud<pcl::PointXYZI> front_left_velodyne_no_ground;      // Pointcloud to save no ground
pcl::PointCloud<pcl::PointXYZI> front_left_velodyne_ground;         // Pointcloud to save ground

pcl::PointCloud<pcl::PointXYZI> rear_right_velodyne_no_ground;      // Pointcloud to save no ground
pcl::PointCloud<pcl::PointXYZI> rear_right_velodyne_ground;         // Pointcloud to save ground

pcl::PointCloud<pcl::PointXYZI> rear_left_velodyne_no_ground;       // Pointcloud to save no ground
pcl::PointCloud<pcl::PointXYZI> rear_left_velodyne_ground;          // Pointcloud to save ground

pcl::PointCloud<pcl::PointXYZI> top_middle_velodyne_no_ground;      // Pointcloud to save no ground
pcl::PointCloud<pcl::PointXYZI> top_middle_velodyne_ground;         // Pointcloud to save ground

pcl::PointCloud<pcl::PointXYZI> livox_no_ground;                    // Pointcloud to save no ground
pcl::PointCloud<pcl::PointXYZI> livox_ground;                       // Pointcloud to save ground

// functions
void getROI(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr);
void getCloudPart(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_part_ptr, const double length, const double deviation);
void removeGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr, const double z_min_ground, const double z_max_ground, const double max_angle);
bool fusePointclouds(pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr);
void voxelgrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud_ptr);
void outlierRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr);
void publishPointcloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr, const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr, const pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud_ptr);

void callbackFrontRight(const pcl::PointCloud<pcl::PointXYZI> input);
void callbackFrontLeft(const pcl::PointCloud<pcl::PointXYZI> input);
void callbackRearRight(const pcl::PointCloud<pcl::PointXYZI> input);
void callbackRearLeft(const pcl::PointCloud<pcl::PointXYZI> input);
void callbackTopMiddle(const pcl::PointCloud<pcl::PointXYZI> input);
void callbackFrintMiddle(const pcl::PointCloud<pcl::PointXYZI> input);
void clearAllPointclouds();
bool allPointcloudsAvailable();

