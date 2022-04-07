// Parameter file
#include "Parameter.h"

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


class PreprocessingNode
{
public:
    /**
     * @brief Construct a new Preprocessing Node object
     * 
     */

    PreprocessingNode(){
         // Debugging
        ROS_INFO("Fusion node is now running");

        // publisher
        fusionpub = node.advertise<pcl::PointCloud<pcl::PointXYZI>>("/cloud_fusion_node/points_fused", 1); // topic name, queue <sensor_msgs::PointCloud2>
        roipub = node.advertise<pcl::PointCloud<pcl::PointXYZI>>("/cloud_fusion_node/points_roi", 1);
        nogroundpub = node.advertise<pcl::PointCloud<pcl::PointXYZI>>("/cloud_fusion_node/points_no_ground", 1);
        groundpub = node.advertise<pcl::PointCloud<pcl::PointXYZI>>("/cloud_fusion_node/points_ground", 1);
        voxelpub = node.advertise<pcl::PointCloud<pcl::PointXYZI>>("/cloud_fusion_node/points_voxel", 1);


        // subscriber
        front_right_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/front_right/velodyne_points", 1, &PreprocessingNode::addFRVelodyne, this);
        front_left_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/front_left/velodyne_points", 1, &PreprocessingNode::addFLVelodyne, this);
        rear_right_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/rear_right/velodyne_points", 1, &PreprocessingNode::addRRVelodyne, this);
        rear_left_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/rear_left/velodyne_points", 1, &PreprocessingNode::addRLVelodyne, this);
        top_middle_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/top_middle/velodyne_points", 1, &PreprocessingNode::addTMVelodyne, this);
        front_livox_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/livoxfront/livox/lidar", 1, &PreprocessingNode::addLiv, this);
    
    }
    /**
     * @brief filter Pointcloud in rectancle shaped ROI
     * 
     * @param cloud_ptr         input cloud
     * @param cloud_ROI_ptr     output cloud as ROI
     */
    void getROI(   const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, 
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr){  

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
                        const double deviation){

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
                        const double max_angle){
        
        // 1.) divide pointcloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_part_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PassThrough<pcl::PointXYZI> zpass;
        zpass.setInputCloud (cloud_ptr);
        zpass.setFilterFieldName ("z");
        zpass.setFilterLimits (z_min_ground, z_max_ground);
        zpass.filter (*ground_cloud_ptr);

        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_part_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PassThrough<pcl::PointXYZI> zpass2;
        zpass2.setInputCloud (cloud_ptr);
        zpass2.setFilterFieldName ("z");
        zpass2.setFilterLimits (z_max_ground + 0.01, roi_z_max);
        zpass2.filter (*no_ground_cloud_ptr);
       
        
        // 2.) RANSAC
        // // preparations
        // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        // pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

        // // Create the segmentation object
        // pcl::SACSegmentation<pcl::PointXYZI> seg;

        // seg.setOptimizeCoefficients (true);
        // // set the modeltype as plane
        // seg.setModelType (pcl::SACMODEL_PLANE);
        // // set the method for segmentation
        // seg.setMethodType (pcl::SAC_RANSAC);
        // // Maximum number of iterations before giving up
        // seg.setMaxIterations (max_iterations);
        // // Set the axis for which to search for perpendicular planes
        // seg.setAxis(axis);
        // // Set maximum allowed angle between the model normal and the given axis in radians
        // seg.setEpsAngle(max_angle); 
        // // Set a allowed defusePointcloudsviati double z_threshold;odel threshold
        // seg.setDistanceThreshold (distance_threshold);
        // seg.setInputCloud (ground_part_ptr);
        // // Set the probability of choosing at least one sample free from outliers. 
        // seg.setProbability(prob);
        // seg.segment (*inliers, *coefficients);

        // // 3.) Extract inliers / outlieres
        // // Create the filtering object
        // pcl::ExtractIndices<pcl::PointXYZI> extract;

        // // Extract the inliers
        // extract.setInputCloud (ground_part_ptr);
        // extract.setIndices (inliers);
        // extract.setNegative (true);
        // extract.filter (*no_ground_cloud_ptr);

        // extract.setNegative(false);
        // extract.filter(*ground_cloud_ptr);

        // // merge clouds

        // *no_ground_cloud_ptr += *no_ground_part_ptr;
    }

    void removeGroundMid(  const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, 
                        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr,
                        const double z_min_ground, 
                        const double z_max_ground){
        

        pcl::PassThrough<pcl::PointXYZI> zpass;
        zpass.setInputCloud (cloud_ptr);
        zpass.setFilterFieldName ("z");
        zpass.setFilterLimits (z_min_ground, z_max_ground);
        zpass.filter (*ground_cloud_ptr);

        pcl::PassThrough<pcl::PointXYZI> zpass2;
        zpass2.setInputCloud (cloud_ptr);
        zpass2.setFilterFieldName ("z");
        zpass2.setFilterLimits (z_max_ground + 0.01, roi_z_max);
        zpass2.filter (*no_ground_cloud_ptr);
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
                            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr){
        // check if all Pointclouds are available without ground
        if (!allPointcloudsAvailable()){
            return false;
        }else{
        
        // fuse pointclouds
        *no_ground_ptr = front_right_velodyne_no_ground;
        *no_ground_ptr += front_left_velodyne_no_ground;
        *no_ground_ptr += rear_right_velodyne_no_ground;
        *no_ground_ptr += rear_left_velodyne_no_ground;
        *no_ground_ptr += top_middle_velodyne_no_ground;
        *no_ground_ptr += livox_no_ground;

        *ground_ptr = front_right_velodyne_ground;
        *ground_ptr += front_left_velodyne_ground;
        *ground_ptr += rear_right_velodyne_ground;
        *ground_ptr += rear_left_velodyne_ground;
        *ground_ptr += top_middle_velodyne_ground;
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
                    pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud_ptr){
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
    void outlierRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr){
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
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud_ptr){
        
        // pointer to pointcloud
        pcl::PointCloud<pcl::PointXYZI> no_ground_cloud;
        pcl::PointCloud<pcl::PointXYZI> ground_cloud;
        pcl::PointCloud<pcl::PointXYZI> voxel_cloud;
        
        no_ground_cloud.header.frame_id = "base_footprint";
        ground_cloud.header.frame_id = "base_footprint";
        voxel_cloud.header.frame_id = "base_footprint";

        no_ground_cloud = * no_ground_ptr;
        ground_cloud = *ground_ptr;
        voxel_cloud = *voxel_cloud_ptr;

        // publish clouds
        nogroundpub.publish(no_ground_cloud);
        groundpub.publish(ground_cloud);
        voxelpub.publish(voxel_cloud);
    }

    // Transforms
    tf::StampedTransform tf_fr;
    tf::StampedTransform tf_fl;
    tf::StampedTransform tf_rr;
    tf::StampedTransform tf_rl;
    tf::StampedTransform tf_tm;
    tf::StampedTransform tf_f_liv;

private:

    // Callback functions 
    /**
     * @brief front right Velodyne: transform + ground removal
     * 
     * @param input raw pointcloud from Sensor
     */
    void addFRVelodyne(const pcl::PointCloud<pcl::PointXYZI> input){ 
        
        // 1.) transform Pointcloud
        tf::Transform transform = tf::Transform(tf_fr.getRotation(), tf_fr.getOrigin());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>); 
        pcl_ros::transformPointCloud(input, *cloud_ptr, transform);

        // 2.) filter ROI
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getROI(cloud_ptr, cloud_ROI_ptr);

        // 3.) remove Ground 3 zones
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI> no_ground;
        pcl::PointCloud<pcl::PointXYZI> ground;

        // 3.1) front
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_front_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getCloudPart(cloud_ROI_ptr, cloud_front_ptr, vf_front_length, vf_mid_length/2);
        removeGround(cloud_front_ptr, no_ground_ptr, ground_ptr, vf_z_min_ground_front, vf_z_max_ground_front, vf_max_angle_front);
        ROS_INFO("max angle front in rad: %f", vf_max_angle_front);
        no_ground = *no_ground_ptr;
        ground = *ground_ptr;
        
        // 3.2) mid
        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mid_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        // getCloudPart(cloud_ROI_ptr, cloud_mid_ptr, vf_mid_length, -vf_mid_length/2);
        // removeGround(cloud_mid_ptr, no_ground_ptr, ground_ptr, vf_z_min_ground_mid, vf_z_max_ground_mid, vf_max_angle_mid);
        // no_ground += *no_ground_ptr;
        // ground += *ground_ptr;
        // 3.2) mid alternativ
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mid_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getCloudPart(cloud_ROI_ptr, cloud_mid_ptr, vf_mid_length, -vf_mid_length/2);
        removeGroundMid(cloud_mid_ptr, no_ground_ptr, ground_ptr, vf_z_min_ground_mid, vf_z_max_ground_mid);
        no_ground += *no_ground_ptr;
        ground += *ground_ptr;

        // 3.3) rear
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rear_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getCloudPart(cloud_ROI_ptr, cloud_rear_ptr, vf_rear_length, -vf_mid_length/2);
        removeGround(cloud_rear_ptr, no_ground_ptr, ground_ptr, vf_z_min_ground_rear, vf_z_max_ground_rear, vf_max_angle_rear);
        no_ground += *no_ground_ptr;
        ground += *ground_ptr;
               
        // 4.) finish
        front_right_velodyne_no_ground = no_ground;
        front_right_velodyne_ground = ground;
        ROS_INFO("processed front right pointcloud");
    }

    /**
     * @brief front left Velodyne: transform + ground removal
     * 
     * @param input raw pointcloud from Sensor
     */
    void addFLVelodyne(const pcl::PointCloud<pcl::PointXYZI> input){ 
        
        // 1.) transform Pointcloud
        tf::Transform transform = tf::Transform(tf_fl.getRotation(), tf_fl.getOrigin());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl_ros::transformPointCloud(input, *cloud_ptr, transform);

        // 2.) filter ROI
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getROI(cloud_ptr, cloud_ROI_ptr);

        // 3.) remove Ground 3 zones
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI> no_ground;
        pcl::PointCloud<pcl::PointXYZI> ground;

        // 3.1) front
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_front_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getCloudPart(cloud_ROI_ptr, cloud_front_ptr, vf_front_length, vf_mid_length/2);
        removeGround(cloud_front_ptr, no_ground_ptr, ground_ptr,vf_z_min_ground_front, vf_z_max_ground_front, vf_max_angle_front);
        no_ground = *no_ground_ptr;
        ground = *ground_ptr;
        
        // 3.2) mid
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mid_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getCloudPart(cloud_ROI_ptr, cloud_mid_ptr, vf_mid_length, -vf_mid_length/2);
        removeGround(cloud_mid_ptr, no_ground_ptr, ground_ptr, vf_z_min_ground_mid, vf_z_max_ground_mid, vf_max_angle_mid);
        no_ground += *no_ground_ptr;
        ground += *ground_ptr;
        // 3.2) mid alternative
        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mid_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        // getCloudPart(cloud_ROI_ptr, cloud_mid_ptr, vf_mid_length, -vf_mid_length/2);
        // removeGroundMid(cloud_mid_ptr, no_ground_ptr, ground_ptr, vf_z_min_ground_mid, vf_z_max_ground_mid);
        // no_ground += *no_ground_ptr;
        // ground += *ground_ptr;

        // 3.3) rear
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rear_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getCloudPart(cloud_ROI_ptr, cloud_rear_ptr, vf_rear_length, -vf_mid_length/2);
        removeGround(cloud_rear_ptr, no_ground_ptr, ground_ptr, vf_z_min_ground_rear, vf_z_max_ground_rear, vf_max_angle_rear);
        no_ground += *no_ground_ptr;
        ground += *ground_ptr;
               
        // 4.) finish
        front_left_velodyne_no_ground = no_ground;
        front_left_velodyne_ground = ground;
        ROS_INFO("processed front left pointcloud");
    }

    /**
     * @brief rear right Velodyne: transform + ground removal
     * 
     * @param input raw pointcloud from Sensor
     */
    void addRRVelodyne(const pcl::PointCloud<pcl::PointXYZI> input){
        
        // 1.) transform Pointcloud
        tf::Transform transform = tf::Transform(tf_rr.getRotation(), tf_rr.getOrigin());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl_ros::transformPointCloud(input, *cloud_ptr, transform);

        // 2.) filter ROI
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getROI(cloud_ptr, cloud_ROI_ptr);

        // 3.) remove Ground 3 zones
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI> no_ground;
        pcl::PointCloud<pcl::PointXYZI> ground;

        // 3.1) front
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_front_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getCloudPart(cloud_ROI_ptr, cloud_front_ptr, vr_front_length, vr_mid_length/2);
        removeGround(cloud_front_ptr, no_ground_ptr, ground_ptr,vr_z_min_ground_front, vr_z_max_ground_front, vr_max_angle_front);
        no_ground = *no_ground_ptr;
        ground = *ground_ptr;
        
        // 3.2) mid
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mid_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getCloudPart(cloud_ROI_ptr, cloud_mid_ptr, vr_mid_length, -vr_mid_length/2);
        removeGround(cloud_mid_ptr, no_ground_ptr, ground_ptr, vr_z_min_ground_mid, vr_z_max_ground_mid, vr_max_angle_mid);
        no_ground += *no_ground_ptr;
        ground += *ground_ptr;
        // 3.2) mid alternative
        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mid_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        // getCloudPart(cloud_ROI_ptr, cloud_mid_ptr, vr_mid_length, -vr_mid_length/2);
        // removeGroundMid(cloud_mid_ptr, no_ground_ptr, ground_ptr, vr_z_min_ground_mid, vr_z_max_ground_mid);
        // no_ground += *no_ground_ptr;
        // ground += *ground_ptr;

        // 3.3) rear
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rear_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getCloudPart(cloud_ROI_ptr, cloud_rear_ptr, vr_rear_length, -vr_mid_length/2);
        removeGround(cloud_rear_ptr, no_ground_ptr, ground_ptr, vr_z_min_ground_rear, vr_z_max_ground_rear, vr_max_angle_rear);
        no_ground += *no_ground_ptr;
        ground += *ground_ptr;
               
        // 4.) finish
        rear_right_velodyne_no_ground = no_ground;
        rear_right_velodyne_ground = ground;
        ROS_INFO("processed rear right pointcloud");
    }

    /**
     * @brief rear left Velodyne: transform + ground removal
     * 
     * @param input raw pointcloud from Sensor
     */
    void addRLVelodyne(const pcl::PointCloud<pcl::PointXYZI> input){ 
        
        // 1.) transform Pointcloud
        tf::Transform transform = tf::Transform(tf_rl.getRotation(), tf_rl.getOrigin());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl_ros::transformPointCloud(input, *cloud_ptr, transform);


        // 2.) filter ROI
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getROI(cloud_ptr, cloud_ROI_ptr);

        // 3.) remove Ground 3 zones
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI> no_ground;
        pcl::PointCloud<pcl::PointXYZI> ground;

        // 3.1) front
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_front_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getCloudPart(cloud_ROI_ptr, cloud_front_ptr, vr_front_length, vr_mid_length/2);
        removeGround(cloud_front_ptr, no_ground_ptr, ground_ptr,vr_z_min_ground_front, vr_z_max_ground_front, vr_max_angle_front);
        no_ground = *no_ground_ptr;
        ground = *ground_ptr;
        
        // 3.2) mid
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mid_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getCloudPart(cloud_ROI_ptr, cloud_mid_ptr, vr_mid_length, -vr_mid_length/2);
        removeGround(cloud_mid_ptr, no_ground_ptr, ground_ptr, l_z_min_ground_mid, l_z_max_ground_mid, l_max_angle_mid);
        no_ground += *no_ground_ptr;
        ground += *ground_ptr;
        // 3.2) mid alternative
        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mid_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        // getCloudPart(cloud_ROI_ptr, cloud_mid_ptr, vr_mid_length, -vr_mid_length/2);
        // removeGroundMid(cloud_mid_ptr, no_ground_ptr, ground_ptr, l_z_min_ground_mid, l_z_max_ground_mid);
        // no_ground += *no_ground_ptr;
        // ground += *ground_ptr;

        // 3.3) rear
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rear_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getCloudPart(cloud_ROI_ptr, cloud_rear_ptr, l_mid_length, 0.0);
        removeGround(cloud_rear_ptr, no_ground_ptr, ground_ptr, l_z_min_ground_mid, vr_z_max_ground_mid, vr_max_angle_mid);
        no_ground += *no_ground_ptr;
        ground += *ground_ptr;
               
        // 4.) finish
        rear_left_velodyne_no_ground = no_ground;
        rear_left_velodyne_ground = ground;
        ROS_INFO("processed rear left pointcloud");
    }

    /**
     * @brief top middle Velodyne: transform + ground removal
     * 
     * @param input raw pointcloud from Sensor
     */
    void addTMVelodyne(const pcl::PointCloud<pcl::PointXYZI> input){
        
        // 1.) transform Pointcloud
        tf::Transform transform = tf::Transform(tf_tm.getRotation(), tf_tm.getOrigin());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl_ros::transformPointCloud(input, *cloud_ptr, transform);

        // 2.) filter ROI
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getROI(cloud_ptr, cloud_ROI_ptr);

        // 3.) remove Ground 2 zones
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI> no_ground;
        pcl::PointCloud<pcl::PointXYZI> ground;

        // 3.1) front
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_front_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getCloudPart(cloud_ROI_ptr, cloud_front_ptr, vt_front_length, vt_mid_length/2);
        removeGround(cloud_front_ptr, no_ground_ptr, ground_ptr,vt_z_min_ground_front, vt_z_max_ground_front, vt_max_angle_front);
        no_ground = *no_ground_ptr;
        ground = *ground_ptr;
        

        // 3.2) rear
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rear_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getCloudPart(cloud_ROI_ptr, cloud_rear_ptr, vt_rear_length, -vt_mid_length/2);
        removeGround(cloud_rear_ptr, no_ground_ptr, ground_ptr, vt_z_min_ground_rear, vt_z_max_ground_rear, vt_max_angle_rear);
        no_ground += *no_ground_ptr;
        ground += *ground_ptr;
               
        // 4.) finish
        top_middle_velodyne_no_ground = no_ground;
        top_middle_velodyne_ground = ground;
        ROS_INFO("processed top middle pointcloud");
    }
    
    /**
     * @brief Livox: transform + ground removal
     * 
     * @param input raw pointcloud from Sensor
     */
    void addLiv(const pcl::PointCloud<pcl::PointXYZI> input){ 
        
        // 1.) transform Pointcloud
        tf::Transform transform = tf::Transform(tf_f_liv.getRotation(), tf_f_liv.getOrigin());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl_ros::transformPointCloud(input, *cloud_ptr, transform);

        // 2.) filter ROI
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getROI(cloud_ptr, cloud_ROI_ptr);

        // 3.) remove Ground 2 zones
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI> no_ground;
        pcl::PointCloud<pcl::PointXYZI> ground;

        // 3.1) front
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_front_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getCloudPart(cloud_ROI_ptr, cloud_front_ptr, l_front_length, l_mid_length);
        removeGround(cloud_front_ptr, no_ground_ptr, ground_ptr, l_z_min_ground_front, l_z_max_ground_front, l_max_angle_front);
        no_ground = *no_ground_ptr;
        ground = *ground_ptr;
        
        // 3.2) mid
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mid_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        getCloudPart(cloud_ROI_ptr, cloud_mid_ptr, vt_rear_length, -vt_mid_length/2);
        removeGround(cloud_mid_ptr, no_ground_ptr, ground_ptr, l_z_min_ground_mid, l_z_max_ground_mid, l_max_angle_mid);
        no_ground += *no_ground_ptr;
        ground += *ground_ptr;
               
        // 4.) finish
        livox_no_ground = no_ground;
        livox_ground = ground;
        ROS_INFO("processed Livox pointcloud");
    }

    /**
     * @brief clear all pointclouds
     * 
     */
    void clearAllPointclouds(){
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
    bool allPointcloudsAvailable(){
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


    // ROS node
    ros::NodeHandle node{ "~" };

    // subscriber
    ros:: Subscriber front_right_sub;
    ros:: Subscriber front_left_sub;
    ros:: Subscriber rear_right_sub;
    ros:: Subscriber rear_left_sub;
    ros:: Subscriber top_middle_sub;
    ros:: Subscriber front_livox_sub;

    // publisher
    ros::Publisher fusionpub;
    ros::Publisher roipub;
    ros::Publisher nogroundpub;
    ros::Publisher groundpub;
    ros::Publisher voxelpub;



};