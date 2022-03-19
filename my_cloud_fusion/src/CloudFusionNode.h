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


class CloudFusionNode
{
public:
    // constructor
    CloudFusionNode(){
        // Debugging
        ROS_INFO("Fusion node is now running");

        // publisher
        fusionpub = node.advertise<pcl::PointCloud<pcl::PointXYZI>>("/cloud_fusion_node/points_fused", 1); // topic name, queue <sensor_msgs::PointCloud2>
        roipub = node.advertise<pcl::PointCloud<pcl::PointXYZI>>("/cloud_fusion_node/points_roi", 1);
        nogroundpub = node.advertise<pcl::PointCloud<pcl::PointXYZI>>("/cloud_fusion_node/points_no_ground", 1);
        groundpub = node.advertise<pcl::PointCloud<pcl::PointXYZI>>("/cloud_fusion_node/points_ground", 1);
        voxelpub = node.advertise<pcl::PointCloud<pcl::PointXYZI>>("/cloud_fusion_node/points_voxel", 1);


        // subscriber
        front_right_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/front_right/velodyne_points", 1, &CloudFusionNode::add_fr_velodyne, this);
        front_left_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/front_left/velodyne_points", 1, &CloudFusionNode::add_fl_velodyne, this);
        rear_right_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/rear_right/velodyne_points", 1, &CloudFusionNode::add_rr_velodyne, this);
        rear_left_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/rear_left/velodyne_points", 1, &CloudFusionNode::add_rl_velodyne, this);
        top_middle_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/top_middle/velodyne_points", 1, &CloudFusionNode::add_tm_velodyne, this);
        front_livox_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/livoxfront/livox/lidar", 1, &CloudFusionNode::add_f_liv, this);
    }
    // cloud fusion
    void cloud_fusion(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr){

        pcl::PointCloud<pcl::PointXYZI> fused_cloud;
        fused_cloud = *cloud_ptr;

        fused_cloud = fr_vel_trans;
        fused_cloud += fl_vel_trans;
        fused_cloud += rr_vel_trans;
        fused_cloud += rl_vel_trans;
        fused_cloud += tm_vel_trans;
        fused_cloud += f_liv_trans;

        *cloud_ptr = fused_cloud;
    }
    // remove outliers
    void remove_outliers(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr){
        
        // build the filter
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
        outrem.setInputCloud(cloud_ptr);
        outrem.setRadiusSearch(radius);
        outrem.setMinNeighborsInRadius(min_neighbor);
        outrem.setKeepOrganized(false);

        // apply filter
        outrem.filter (*cloud_ptr);
    }
    // filter ROI 
    void filter_ROI_T(    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, 
                        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr){

        //build the  |, first part
        pcl::PointCloud<pcl::PointXYZI>::Ptr first_part_ptr( new pcl::PointCloud<pcl::PointXYZI>);                   
        // filter in x
        pcl::PassThrough<pcl::PointXYZI> xpass;
        xpass.setInputCloud (cloud_ptr);
        xpass.setFilterFieldName ("x");
        xpass.setFilterLimits (-x_longitudinal/2, x_longitudinal/2);
        xpass.filter (*first_part_ptr);

        // filter in y
        pcl::PassThrough<pcl::PointXYZI> ypass;
        ypass.setInputCloud (first_part_ptr);
        ypass.setFilterFieldName ("y");
        ypass.setFilterLimits (-y_longitudinal/2, y_longitudinal/2);
        ypass.filter (*first_part_ptr);

        // filter in z
        pcl::PassThrough<pcl::PointXYZI> zpass;
        zpass.setInputCloud (first_part_ptr);
        zpass.setFilterFieldName ("z");
        zpass.setFilterLimits (z_min, z_max);
        zpass.filter (*first_part_ptr);

        // build the --, secound part
        pcl::PointCloud<pcl::PointXYZI>::Ptr secound_part_ptr (new pcl::PointCloud<pcl::PointXYZI>); 
        // filter in x
        pcl::PassThrough<pcl::PointXYZI> xpass2;
        xpass2.setInputCloud (cloud_ptr);
        xpass2.setFilterFieldName ("x");
        xpass2.setFilterLimits (x_longitudinal/2, (x_longitudinal/2)+x_transverse);
        xpass2.filter (*secound_part_ptr);

        // filter in y
        pcl::PassThrough<pcl::PointXYZI> ypass2;
        ypass2.setInputCloud (secound_part_ptr);
        ypass2.setFilterFieldName ("y");
        ypass2.setFilterLimits (-y_transverse/2, y_transverse/2);
        ypass2.filter (*secound_part_ptr);

        // filter in z
        pcl::PassThrough<pcl::PointXYZI> zpass2;
        zpass2.setInputCloud (secound_part_ptr);
        zpass2.setFilterFieldName ("z");
        zpass2.setFilterLimits (z_min, z_max);
        zpass2.filter (*secound_part_ptr);

        // Fuse both segments
        pcl::PointCloud<pcl::PointXYZI> first_part;
        first_part = *first_part_ptr;
        pcl::PointCloud<pcl::PointXYZI> secound_part;
        secound_part = *secound_part_ptr;
        first_part += secound_part;
        *filtered_cloud_ptr = first_part;
    }
    // filter ROI as rectangle
    void filter_ROI_R(    const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, 
                        pcl::PointCloud<pcl::PointXYZI>::Ptr front_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr mid_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr rear_cloud_ptr){
        
        //build the  |, first part
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_ptr( new pcl::PointCloud<pcl::PointXYZI>);  

        // filter in height
        pcl::PassThrough<pcl::PointXYZI> zpass;
        zpass.setInputCloud (cloud_ptr);
        zpass.setFilterFieldName ("z");
        zpass.setFilterLimits (z_min, z_max);
        zpass.filter (*filtered_ptr);

        // filter in width
        pcl::PassThrough<pcl::PointXYZI> ypass;
        ypass.setInputCloud (filtered_ptr);
        ypass.setFilterFieldName ("y");
        ypass.setFilterLimits (-lane_width/2, lane_width/2);
        ypass.filter (*filtered_ptr);

        // filter front cloud
        pcl::PassThrough<pcl::PointXYZI> front_range;
        front_range.setInputCloud (filtered_ptr);
        front_range.setFilterFieldName ("x");
        front_range.setFilterLimits (mid_length/2, (mid_length/2)+front_length);
        front_range.filter (*front_cloud_ptr);

        // filter mid cloud
        pcl::PassThrough<pcl::PointXYZI> mid_range;
        mid_range.setInputCloud (filtered_ptr);
        mid_range.setFilterFieldName ("x");
        mid_range.setFilterLimits (-mid_length/2, mid_length/2);
        mid_range.filter (*mid_cloud_ptr);
        
        // filter rear cloud
        pcl::PassThrough<pcl::PointXYZI> rear_range;
        rear_range.setInputCloud (filtered_ptr);
        rear_range.setFilterFieldName ("x");
        rear_range.setFilterLimits (-(mid_length/2) - rear_length, rear_length);
        rear_range.filter (*rear_cloud_ptr);
    }
    // remove ground with RANSAC 
    void remove_ground( const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, 
                        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr,
                        const double z_min_ground, 
                        const double z_max_ground,
                        const double max_angle){
        
        // divide pointcloud
        // -----------------------------------------------------------------------------------------
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
        zpass2.setFilterLimits (z_max_ground + 0.01, z_max);
        zpass2.filter (*no_ground_part_ptr);
        // -----------------------------------------------------------------------------------------
        
        // RANSAC
        // -----------------------------------------------------------------------------------------
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
        // Set a allowed deviati double z_threshold;odel threshold
        seg.setDistanceThreshold (distance_threshold);
        seg.setInputCloud (ground_part_ptr);
        // Set the probability of choosing at least one sample free from outliers. 
        seg.setProbability(prob);
        seg.segment (*inliers, *coefficients);
        // -----------------------------------------------------------------------------------------
        // Extract inliers / outlieres
        // -----------------------------------------------------------------------------------------
        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZI> extract;

        // Extract the inliers
        extract.setInputCloud (ground_part_ptr);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (*no_ground_cloud_ptr);

        extract.setNegative(false);
        extract.filter(*ground_cloud_ptr);
        // -----------------------------------------------------------------------------------------

        // merge clouds
        // -----------------------------------------------------------------------------------------
        pcl::PointCloud<pcl::PointXYZI> no_ground;
        no_ground = *no_ground_cloud_ptr;
        pcl::PointCloud<pcl::PointXYZI> no_ground_part;
        no_ground_part = *no_ground_part_ptr;

        no_ground += no_ground_part;

        *no_ground_cloud_ptr = no_ground;
        // -----------------------------------------------------------------------------------------

    }
    // voxelgrid filter
    void voxelgrid( const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud_ptr){

        // build the filter
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
        voxel_grid.setInputCloud (cloud);
        // set Size of each voxel in x, y, z
        voxel_grid.setLeafSize (voxel_size, voxel_size, voxel_size);
        // Set to true if all fields need to be downsampled, or false if just XYZ.
        voxel_grid.setDownsampleAllData(true);
        // Set the minimum number of points required for a voxel to be used
        voxel_grid.setMinimumPointsNumberPerVoxel(points_per_voxel);   
        voxel_grid.filter(*voxel_cloud_ptr);
    }
    // publish clouds
    void publish(   const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr,
                    const pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud_ptr){
        
        // get clouds from pointers
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::PointCloud<pcl::PointXYZI> filtered_cloud;
        pcl::PointCloud<pcl::PointXYZI> no_ground_cloud;
        pcl::PointCloud<pcl::PointXYZI> ground_cloud;
        pcl::PointCloud<pcl::PointXYZI> voxel_cloud;

        cloud = *cloud_ptr;
        filtered_cloud = *filtered_cloud_ptr;
        no_ground_cloud = *no_ground_cloud_ptr;
        ground_cloud = *ground_cloud_ptr;
        voxel_cloud = *voxel_cloud_ptr;

        // set default values 
        cloud.header.frame_id = "base_footprint";
        filtered_cloud.header.frame_id = "base_footprint";
        no_ground_cloud.header.frame_id = "base_footprint";
        ground_cloud.header.frame_id = "base_footprint";
        voxel_cloud.header.frame_id = "base_footprint";

        // publish fused points
        fusionpub.publish(cloud);
        roipub.publish(filtered_cloud);
        nogroundpub.publish(no_ground_cloud);
        groundpub.publish(ground_cloud);
        voxelpub.publish(voxel_cloud);
    }

    
    // proceed single Pointcloud
    void proceed_pointcloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr,
                            const int index){
        
        // 1.) fuse all input clouds
        switch (index){
            case 0 :
                *cloud_ptr = fr_vel_trans;
                break;
            case 1 :
                *cloud_ptr = fl_vel_trans;
                break;
            case 2 :
                *cloud_ptr = rr_vel_trans;
                break;
            case 3 :
                *cloud_ptr = rl_vel_trans;
                break;
            case 4 :
                *cloud_ptr = tm_vel_trans;
                break;
            case 5 :
                *cloud_ptr = f_liv_trans;
                break;

        }       
              
        // 2.) filter ROI rectangle
        pcl::PointCloud<pcl::PointXYZI>::Ptr front_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr mid_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr rear_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        filter_ROI_R(cloud_ptr,front_cloud_ptr, mid_cloud_ptr, rear_cloud_ptr);

        // remove ground front
        remove_ground(front_cloud_ptr, no_ground_cloud_ptr, ground_cloud_ptr, z_min_ground_front, z_max_ground_front, max_angle_front);
        pcl::PointCloud<pcl::PointXYZI> ground1;
        ground1 = *ground_cloud_ptr;
        pcl::PointCloud<pcl::PointXYZI> noground1;
        noground1 = *no_ground_cloud_ptr;

        // remove ground mid
        remove_ground(mid_cloud_ptr, no_ground_cloud_ptr, ground_cloud_ptr, z_min_ground_mid, z_max_ground_mid, max_angle_mid);
        pcl::PointCloud<pcl::PointXYZI> ground2;
        ground2 = *ground_cloud_ptr;
        pcl::PointCloud<pcl::PointXYZI> noground2;
        noground2 = *no_ground_cloud_ptr;
        
        // remove ground rear
        remove_ground(rear_cloud_ptr, no_ground_cloud_ptr, ground_cloud_ptr, z_min_ground_rear, z_max_ground_rear, max_angle_rear);
        pcl::PointCloud<pcl::PointXYZI> ground3;
        ground3 = *ground_cloud_ptr;
        pcl::PointCloud<pcl::PointXYZI> noground3;
        noground3 = *no_ground_cloud_ptr;

        // merge Clouds
        ground1 += ground2;
        ground1 += ground3;
        *ground_cloud_ptr = ground1;

        noground1 += noground2;
        noground1 += noground3;
        *no_ground_cloud_ptr = noground1;

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
    void add_fr_velodyne(const pcl::PointCloud<pcl::PointXYZI> input){ // ::ConstPtr& 
        tf::Transform transform = tf::Transform(tf_fr.getRotation(), tf_fr.getOrigin());
        pcl_ros::transformPointCloud(input, fr_vel_trans, transform);
    }

    void add_fl_velodyne(const pcl::PointCloud<pcl::PointXYZI> input){ 
        tf::Transform transform = tf::Transform(tf_fl.getRotation(), tf_fl.getOrigin());
        pcl_ros::transformPointCloud(input, fl_vel_trans, transform);
    }

    void add_rr_velodyne(const pcl::PointCloud<pcl::PointXYZI> input){
        tf::Transform transform = tf::Transform(tf_rr.getRotation(), tf_rr.getOrigin());
        pcl_ros::transformPointCloud(input, rr_vel_trans, transform);
    }

    void add_rl_velodyne(const pcl::PointCloud<pcl::PointXYZI> input){ 
        tf::Transform transform = tf::Transform(tf_rl.getRotation(), tf_rl.getOrigin());
        pcl_ros::transformPointCloud(input, rl_vel_trans, transform);
    }

    void add_tm_velodyne(const pcl::PointCloud<pcl::PointXYZI> input){
        tf::Transform transform = tf::Transform(tf_tm.getRotation(), tf_tm.getOrigin());
        pcl_ros::transformPointCloud(input, tm_vel_trans, transform);
    }

    void add_f_liv(const pcl::PointCloud<pcl::PointXYZI> input){ 
        tf::Transform transform = tf::Transform(tf_f_liv.getRotation(), tf_f_liv.getOrigin());
        pcl_ros::transformPointCloud(input, f_liv_trans, transform);
    }

    // Pointclouds
    pcl::PointCloud<pcl::PointXYZI> fr_vel_trans;
    pcl::PointCloud<pcl::PointXYZI> fl_vel_trans;
    pcl::PointCloud<pcl::PointXYZI> rr_vel_trans;
    pcl::PointCloud<pcl::PointXYZI> rl_vel_trans;
    pcl::PointCloud<pcl::PointXYZI> tm_vel_trans;
    pcl::PointCloud<pcl::PointXYZI> f_liv_trans;

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

    // create msgs
    std_msgs::String debugmsg;

};