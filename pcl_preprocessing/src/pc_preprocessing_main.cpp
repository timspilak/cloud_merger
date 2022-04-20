/**
 * @file pc_preprocessing_main.cpp
 * @author Tim Spilak (tspilak@stud.hs-heilbronn.de)
 * @brief 
 * @version 0.1
 * @date 2022-04-14
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "pc_preprocessing_main.h"
#include "Parameter.h"

/**
 * @brief filter Pointcloud in rectancle shaped ROI
 * 
 * @param cloud_ptr         input cloud
 * @param cloud_ROI_ptr     output cloud as ROI
 */
void getROI(   const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, 
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr)
{  
    // filter in height
    pcl::PassThrough<pcl::PointXYZI> zpass;
    zpass.setInputCloud (cloud_ptr);
    zpass.setFilterFieldName ("z");
    zpass.setFilterLimits (roi_z_min, roi_z_max);
    zpass.filter (*cloud_ROI_ptr);

    // filter in width
    pcl::PassThrough<pcl::PointXYZI> ypass;
    ypass.setInputCloud (cloud_ROI_ptr);
    ypass.setFilterFieldName ("y");
    ypass.setFilterLimits (-roi_width/2, roi_width/2);
    ypass.filter (*cloud_ROI_ptr);

    // filter in length
    pcl::PassThrough<pcl::PointXYZI> xpass;
    xpass.setInputCloud (cloud_ROI_ptr);
    xpass.setFilterFieldName ("x");
    xpass.setFilterLimits (-roi_mid, roi_length-roi_mid);
    xpass.filter (*cloud_ROI_ptr);

}
/**
 * @brief Get the cloud part object
 * 
 * @param cloud_ptr         input cloud
 * @param cloud_part        output cloud that is a part of input
 * @param length            length of the part in [ m ]
 * @param deviation         distance from center of vehicle to start
 */
void getCloudPart(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, 
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_part_ptr,
                    const double length,
                    const double deviation)
{

    // filter front cloud
    pcl::PassThrough<pcl::PointXYZI> range;
    range.setInputCloud (cloud_ptr);
    range.setFilterFieldName ("x");
    range.setFilterLimits (deviation, deviation+length);
    range.filter (*cloud_part_ptr);
}
/**
 * @brief Ground removal via RANSAC-Algorithm
 * 
 * @param cloud_ptr             input cloud
 * @param no_ground_cloud_ptr   output cloud that is no ground
 * @param ground_cloud_ptr      output cloud that is the ground
 * @param z_min_ground          minimum z to search ground in [ m ]
 * @param z_max_ground          maximum z to search ground in [ m ]
 * @param max_angle             maximum angle of the searched plane in [ rad ]
 */

void removeGround(  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, 
                    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr,
                    const double z_min_ground, 
                    const double z_max_ground,
                    const double max_angle)
{
    
    // 1.) divide pointcloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_part_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> zpass;
    zpass.setInputCloud (cloud_ptr);
    zpass.setFilterFieldName ("z");
    zpass.setFilterLimits (z_min_ground, z_max_ground);
    zpass.filter (*ground_part_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_part_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> zpass2;
    zpass2.setInputCloud (cloud_ptr);
    zpass2.setFilterFieldName ("z");
    zpass2.setFilterLimits (z_max_ground + 0.01, roi_z_max);
    zpass2.filter (*no_ground_part_ptr);
    
    
    // 2.) RANSAC
    // preparations
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZI> seg;

    seg.setOptimizeCoefficients (true);
    // set the modeltype as plane
    seg.setModelType (pcl::SACMODEL_PLANE);
    // set the method for segmentation
    seg.setMethodType (pcl::SAC_RANSAC);
    // Maximum number of iterations before giving up
    seg.setMaxIterations (max_iterations);
    // Set the axis for which to search for perpendicular planes
    seg.setAxis(axis);
    // Set maximum allowed angle between the model normal and the given axis in radians
    seg.setEpsAngle(max_angle); 
    // Set a allowed defusePointcloudsviati double z_threshold;odel threshold
    seg.setDistanceThreshold (distance_threshold);
    seg.setInputCloud (ground_part_ptr);
    // Set the probability of choosing at least one sample free from outliers. 
    seg.setProbability(prob);
    seg.segment (*inliers, *coefficients);

    // 3.) Extract inliers / outlieres
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZI> extract;

    // Extract the inliers
    extract.setInputCloud (ground_part_ptr);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*no_ground_cloud_ptr);

    extract.setNegative(false);
    extract.filter(*ground_cloud_ptr);

    // merge clouds

    *no_ground_cloud_ptr += *no_ground_part_ptr;
}
/**
 * @brief fuse all pointclouds if they are available
 * 
 * @param no_ground_ptr container of fused clouds that are no ground   
 * @param ground_ptr    container of fused clouds that are ground
 * @return true         if successful fusion
 * @return false        if not all clouds were available
 */
bool fusePointclouds(   pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr)
{
    // check if all Pointclouds are available without ground
    if (!allPointcloudsAvailable()){
        return false;
    }else{
    initClouds();
    // fuse pointclouds
    // *no_ground_ptr = front_right_velodyne_no_ground;
    // *no_ground_ptr += front_left_velodyne_no_ground;
    // *no_ground_ptr += rear_right_velodyne_no_ground;
    // *no_ground_ptr += rear_left_velodyne_no_ground;
    // *no_ground_ptr += top_middle_velodyne_no_ground;
    *no_ground_ptr += livox_no_ground;

    // *ground_ptr = front_right_velodyne_ground;
    // *ground_ptr += front_left_velodyne_ground;
    // *ground_ptr += rear_right_velodyne_ground;
    // *ground_ptr += rear_left_velodyne_ground;
    // *ground_ptr += top_middle_velodyne_ground;
    *ground_ptr += livox_ground;

    // clear pointclouds
    clearAllPointclouds();
        return true;
    }
}

/**
 * @brief VoxelGrid filter
 * 
 * @param cloud_ptr         input cloud
 * @param voxel_cloud_ptr   filtered cloud
 */
void voxelgrid(  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr,
                pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud_ptr)
{
    // build the filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setInputCloud (cloud_ptr);
    // set Size of each voxel in x, y, z
    voxel_grid.setLeafSize (voxel_size, voxel_size, voxel_size);
    // Set to true if all fields need to be downsampled, or false if just XYZ.
    voxel_grid.setDownsampleAllData(true);
    // Set the minimum number of points required for a voxel to be used
    voxel_grid.setMinimumPointsNumberPerVoxel(points_per_voxel);   
    voxel_grid.filter(*voxel_cloud_ptr);

}

/**
 * @brief Radius outlier removal
 * 
 * @param cloud_ptr clou to filter outliers
 */
void outlierRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr)
{
    // build the filter
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    outrem.setInputCloud(cloud_ptr);
    outrem.setRadiusSearch(radius);
    outrem.setMinNeighborsInRadius(min_neighbor);
    outrem.setKeepOrganized(false);

    // apply filter
    outrem.filter (*cloud_ptr);
}
/**
 * @brief publish clouds
 * 
 * @param no_ground_ptr cloud that is no ground
 * @param ground_ptr    cloud that represents the ground
 */
void publishPointcloud( const pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud_ptr)
{
    sensor_msgs::PointCloud2 no_ground_msg;
    pcl::toROSMsg(*no_ground_ptr, no_ground_msg);
    no_ground_msg.header.stamp = ros::Time::now();
    no_ground_msg.header.frame_id = "base_footprint";
    nogroundpub.publish(no_ground_msg);

    sensor_msgs::PointCloud2 ground_msg;
    pcl::toROSMsg(*ground_ptr, ground_msg);
    ground_msg.header.stamp = ros::Time::now();
    ground_msg.header.frame_id = "base_footprint";
    groundpub.publish(ground_msg);

    sensor_msgs::PointCloud2 voxel_msg;
    pcl::toROSMsg(*voxel_cloud_ptr, voxel_msg);
    voxel_msg.header.stamp = ros::Time::now();
    voxel_msg.header.frame_id = "base_footprint";
    voxelpub.publish(voxel_msg);
}
/**
 * @brief front right Velodyne: transform + ground removal
 * 
 * @param input raw pointcloud from Sensor
 */
void callbackFrontRight(const pcl::PointCloud<pcl::PointXYZI> input)
{     
    tf::Transform transform = tf::Transform(front_right_stf.getRotation(), front_right_stf.getOrigin());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>); 
    pcl_ros::transformPointCloud(input, *cloud_ptr, transform);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getROI(cloud_ptr, cloud_ROI_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> no_ground;
    pcl::PointCloud<pcl::PointXYZI> ground;

    // front
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_front_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getCloudPart(cloud_ROI_ptr, cloud_front_ptr, vf_front_length, vf_deviation_mid_point);
    removeGround(cloud_front_ptr, no_ground_ptr, ground_ptr, vf_z_min_ground_front, vf_z_max_ground_front, max_angle);
    no_ground = *no_ground_ptr;
    ground = *ground_ptr;
    
    // rear
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rear_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getCloudPart(cloud_ROI_ptr, cloud_rear_ptr, vf_rear_length, -vf_rear_length + vf_deviation_mid_point);
    removeGround(cloud_rear_ptr, no_ground_ptr, ground_ptr, vf_z_min_ground_rear, vf_z_max_ground_rear, max_angle);
    no_ground += *no_ground_ptr;
    ground += *ground_ptr;
    
    // add no ground zone (front)
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_no_ground_part_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getCloudPart(cloud_ROI_ptr, cloud_no_ground_part_ptr, roi_length-roi_mid-vf_front_length-vf_deviation_mid_point, vf_front_length + vf_deviation_mid_point);
    no_ground += *cloud_no_ground_part_ptr;
    
    front_right_velodyne_no_ground = no_ground;
    front_right_velodyne_ground = ground;
    ROS_INFO("processed front right pointcloud");
}

/**
 * @brief front left Velodyne: transform + ground removal
 * 
 * @param input raw pointcloud from Sensor
 */
void callbackFrontLeft(const pcl::PointCloud<pcl::PointXYZI> input)
{     
    tf::Transform transform = tf::Transform(front_left_stf.getRotation(), front_left_stf.getOrigin());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl_ros::transformPointCloud(input, *cloud_ptr, transform);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getROI(cloud_ptr, cloud_ROI_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> no_ground;
    pcl::PointCloud<pcl::PointXYZI> ground;

    // front
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_front_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getCloudPart(cloud_ROI_ptr, cloud_front_ptr, vf_front_length, vf_deviation_mid_point);
    removeGround(cloud_front_ptr, no_ground_ptr, ground_ptr,vf_z_min_ground_front, vf_z_max_ground_front, max_angle);
    no_ground = *no_ground_ptr;
    ground = *ground_ptr;

    // rear
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rear_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getCloudPart(cloud_ROI_ptr, cloud_rear_ptr, vf_rear_length, -vf_rear_length + vf_deviation_mid_point);
    removeGround(cloud_rear_ptr, no_ground_ptr, ground_ptr, vf_z_min_ground_rear, vf_z_max_ground_rear, max_angle);
    no_ground += *no_ground_ptr;
    ground += *ground_ptr;
            
    // add no ground zone
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_no_ground_part_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getCloudPart(cloud_ROI_ptr, cloud_no_ground_part_ptr, roi_length-roi_mid-vf_front_length-vf_deviation_mid_point, vf_front_length + vf_deviation_mid_point);
    no_ground += *cloud_no_ground_part_ptr;
    
    //pass pointclouds
    front_left_velodyne_no_ground = no_ground;
    front_left_velodyne_ground = ground;
    ROS_INFO("processed front left pointcloud");
}

/**
 * @brief rear right Velodyne: transform + ground removal
 * 
 * @param input raw pointcloud from Sensor
 */
void callbackRearRight(const pcl::PointCloud<pcl::PointXYZI> input)
{
    tf::Transform transform = tf::Transform(rear_right_stf.getRotation(), rear_right_stf.getOrigin());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl_ros::transformPointCloud(input, *cloud_ptr, transform);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getROI(cloud_ptr, cloud_ROI_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> no_ground;
    pcl::PointCloud<pcl::PointXYZI> ground;

    // front
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_front_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getCloudPart(cloud_ROI_ptr, cloud_front_ptr, vr_front_length, vr_deviation_mid_point);
    removeGround(cloud_front_ptr, no_ground_ptr, ground_ptr,vr_z_min_ground_front, vr_z_max_ground_front, max_angle);
    no_ground = *no_ground_ptr;
    ground = *ground_ptr;

    // rear
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rear_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getCloudPart(cloud_ROI_ptr, cloud_rear_ptr, vr_rear_length,  vr_deviation_mid_point - vr_rear_length);
    removeGround(cloud_rear_ptr, no_ground_ptr, ground_ptr, vr_z_min_ground_rear, vr_z_max_ground_rear, max_angle);
    no_ground += *no_ground_ptr;
    ground += *ground_ptr;

    // add no ground zone
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_no_ground_part_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getCloudPart(cloud_ROI_ptr, cloud_no_ground_part_ptr, roi_length-roi_mid-vr_front_length+vr_deviation_mid_point, vr_front_length + vr_deviation_mid_point);
    no_ground += *cloud_no_ground_part_ptr;

    // pass pointclouds
    rear_right_velodyne_no_ground = no_ground;
    rear_right_velodyne_ground = ground;
    ROS_INFO("processed rear right pointcloud");
}

/**
 * @brief rear left Velodyne: transform + ground removal
 * 
 * @param input raw pointcloud from Sensor
 */
void callbackRearLeft(const pcl::PointCloud<pcl::PointXYZI> input)
{ 
    tf::Transform transform = tf::Transform(rear_left_stf.getRotation(), rear_left_stf.getOrigin());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl_ros::transformPointCloud(input, *cloud_ptr, transform);


    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getROI(cloud_ptr, cloud_ROI_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> no_ground;
    pcl::PointCloud<pcl::PointXYZI> ground;

    // front
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_front_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getCloudPart(cloud_ROI_ptr, cloud_front_ptr, vr_front_length, 0.0);
    removeGround(cloud_front_ptr, no_ground_ptr, ground_ptr,vr_z_min_ground_front, vr_z_max_ground_front, max_angle);
    no_ground = *no_ground_ptr;
    ground = *ground_ptr;

    // rear
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rear_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getCloudPart(cloud_ROI_ptr, cloud_rear_ptr, vr_rear_length, vr_deviation_mid_point - vr_rear_length);
    removeGround(cloud_rear_ptr, no_ground_ptr, ground_ptr, vr_z_min_ground_rear, vr_z_max_ground_rear, max_angle);
    no_ground += *no_ground_ptr;
    ground += *ground_ptr;

    // add no ground zone
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_no_ground_part_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getCloudPart(cloud_ROI_ptr, cloud_no_ground_part_ptr, roi_length-roi_mid-vr_front_length+vr_deviation_mid_point, vr_front_length + vr_deviation_mid_point);
    no_ground += *cloud_no_ground_part_ptr;
            
    // pass pointclouds
    rear_left_velodyne_no_ground = no_ground;
    rear_left_velodyne_ground = ground;
    ROS_INFO("processed rear left pointcloud");
}
/**
 * @brief top middle Velodyne: transform + ground removal
 * 
 * @param input raw pointcloud from Sensor
 */
void callbackTopMiddle(const pcl::PointCloud<pcl::PointXYZI> input)
{
    tf::Transform transform = tf::Transform(top_middle_stf.getRotation(), top_middle_stf.getOrigin());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl_ros::transformPointCloud(input, *cloud_ptr, transform);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getROI(cloud_ptr, cloud_ROI_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> no_ground;
    pcl::PointCloud<pcl::PointXYZI> ground;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_front_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getCloudPart(cloud_ROI_ptr, cloud_front_ptr, vt_front_length, vt_deviation_start_point);
    removeGround(cloud_front_ptr, no_ground_ptr, ground_ptr, vt_z_min_ground_front, vt_z_max_ground_front, max_angle);
    no_ground = *no_ground_ptr;
    ground = *ground_ptr;

    // add no ground zone
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_no_ground_part_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getCloudPart(cloud_ROI_ptr, cloud_no_ground_part_ptr, roi_mid + vt_deviation_start_point, -roi_mid);
    no_ground += *cloud_no_ground_part_ptr;

    // pointclouds
    top_middle_velodyne_no_ground = no_ground;
    top_middle_velodyne_ground = ground;
    ROS_INFO("processed top middle pointcloud");
}

/**
 * @brief Livox: transform + ground removal
 * 
 * @param input raw pointcloud from Sensor
 */
void callbackFrontMiddle(const pcl::PointCloud<pcl::PointXYZI> input)
{ 
    tf::Transform transform = tf::Transform(front_middle_stf.getRotation(), front_middle_stf.getOrigin());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl_ros::transformPointCloud(input, *cloud_ptr, transform);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getROI(cloud_ptr, cloud_ROI_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI> no_ground;
    pcl::PointCloud<pcl::PointXYZI> ground;

    // front
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_front_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getCloudPart(cloud_ROI_ptr, cloud_front_ptr, l_front_length, l_deviation_mid_point + l_rear_length + l_mid_length);
    removeGround(cloud_front_ptr, no_ground_ptr, ground_ptr, l_z_min_ground_front, l_z_max_ground_front, max_angle);
    no_ground = *no_ground_ptr;
    ground = *ground_ptr;
    
    // mid
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mid_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getCloudPart(cloud_ROI_ptr, cloud_mid_ptr, l_mid_length, l_deviation_mid_point + l_rear_length);
    removeGround(cloud_mid_ptr, no_ground_ptr, ground_ptr, l_z_min_ground_mid, l_z_max_ground_mid, max_angle);
    no_ground += *no_ground_ptr;
    ground += *ground_ptr;

    // rear
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rear_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    getCloudPart(cloud_ROI_ptr, cloud_rear_ptr, l_rear_length, l_deviation_mid_point);
    removeGround(cloud_rear_ptr, no_ground_ptr, ground_ptr, l_z_min_ground_rear, l_z_max_ground_rear, max_angle);
    no_ground += *no_ground_ptr;
    ground += *ground_ptr;
    
    *no_ground_ptr = no_ground;
    outlierRemoval(no_ground_ptr);
    // pass pointclouds
    livox_no_ground = *no_ground_ptr;
    livox_ground = ground;
    ROS_INFO("processed Livox pointcloud");
}
/**
 * @brief clear all pointclouds
 * 
 */
void clearAllPointclouds()
{
    front_right_velodyne_no_ground.clear();
    front_left_velodyne_no_ground.clear();
    front_right_velodyne_ground.clear();
    front_left_velodyne_ground.clear();
    rear_right_velodyne_ground.clear();
    rear_right_velodyne_no_ground.clear();
    rear_left_velodyne_ground.clear();
    rear_left_velodyne_no_ground.clear();
    top_middle_velodyne_ground.clear();
    top_middle_velodyne_no_ground.clear();
    livox_ground.clear();
    livox_no_ground.clear();
}

/**
 * @brief check if each pointcloud was received, transformed and ground removed
 * 
 * @return true if everey Pointcloud was processed
 * @return false if not everey Pointcloud was processed
 */
bool allPointcloudsAvailable()
{
    if( !front_right_velodyne_no_ground.empty()&&
        !front_right_velodyne_ground.empty()&&
        !front_left_velodyne_no_ground.empty()&&
        !front_left_velodyne_ground.empty()&&
        !rear_right_velodyne_no_ground.empty()&&
        !rear_right_velodyne_ground.empty()&&
        !rear_left_velodyne_no_ground.empty()&&
        !rear_left_velodyne_ground.empty()&&
        !livox_no_ground.empty()&&
        !livox_ground.empty())
    {
        return true;
    }
    else
    {
        return false;
    }
    // top middle is not in everey bag available
    //!top_middle_velodyne_no_ground.empty()&&
    //!top_middle_velodyne_ground.empty()&&
}

void initClouds()
{
    front_right_velodyne_no_ground.header.frame_id = "base_footprint";
    front_right_velodyne_ground.header.frame_id = "base_footprint";
    front_left_velodyne_no_ground.header.frame_id = "base_footprint";
    front_left_velodyne_ground.header.frame_id = "base_footprint";
    rear_right_velodyne_no_ground.header.frame_id = "base_footprint";
    rear_right_velodyne_ground.header.frame_id = "base_footprint";
    rear_left_velodyne_no_ground.header.frame_id = "base_footprint";
    rear_left_velodyne_ground.header.frame_id = "base_footprint";
    top_middle_velodyne_no_ground.header.frame_id = "base_footprint";
    top_middle_velodyne_ground.header.frame_id = "base_footprint";
    livox_no_ground.header.frame_id = "base_footprint";
    livox_ground.header.frame_id = "base_footprint";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PointcloudPreprocessing");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(6);
    spinner.start();
    
    nogroundpub = nh.advertise<sensor_msgs::PointCloud2>("/points_no_ground", 1);
    groundpub = nh.advertise<sensor_msgs::PointCloud2>("/points_ground", 1);
    voxelpub = nh.advertise<sensor_msgs::PointCloud2>("/points_voxel", 1);

    ros::Subscriber front_right_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/front_right/velodyne_points", 0, callbackFrontRight);
    ros::Subscriber front_left_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/front_left/velodyne_points", 0, callbackFrontLeft);
    ros::Subscriber rear_right_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/rear_right/velodyne_points", 0, callbackRearRight);
    ros::Subscriber rear_left_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/rear_left/velodyne_points", 0, callbackRearLeft);
    ros::Subscriber top_middle_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/top_middle/velodyne_points", 0, callbackTopMiddle);
    ros::Subscriber front_livox_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/livoxfront/livox/lidar", 0, callbackFrontMiddle);
    
    ros::Rate loop_rate(10);
    tf::TransformListener listener_fr;
    tf::TransformListener listener_fl;
    tf::TransformListener listener_rr;
    tf::TransformListener listener_rl;
    tf::TransformListener listener_tm;
    tf::TransformListener listener_f_liv;   

    while (ros::ok())
    {
        if (!transformations_received)
        {
            // get all Transformations
            try
            {
                listener_fr.lookupTransform("/base_footprint","/velodyne_front_right", ros::Time(0), front_right_stf);
                listener_fl.lookupTransform("/base_footprint","/velodyne_front_left", ros::Time(0), front_left_stf);
                listener_rr.lookupTransform("/base_footprint","/velodyne_rear_right", ros::Time(0), rear_right_stf);
                listener_rl.lookupTransform("/base_footprint","/velodyne_rear_left", ros::Time(0), rear_left_stf);
                listener_tm.lookupTransform("/base_footprint", "/velodyne_top_middle", ros::Time(0), top_middle_stf);
                listener_f_liv.lookupTransform("/base_footprint","/livox_front", ros::Time(0), front_middle_stf);
                transformations_received = true;
            }
            catch(tf::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
            }
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);

        fusePointclouds(no_ground_ptr, ground_ptr);
        // voxelgrid(no_ground_ptr,voxel_cloud_ptr);
        publishPointcloud(no_ground_ptr, ground_ptr, voxel_cloud_ptr);
        
        // ros::spinOnce();
        loop_rate.sleep();
    }
    ros::waitForShutdown();
    return 0;
}