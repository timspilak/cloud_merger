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
        front_right_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/front_right/velodyne_points", 1, &PreprocessingNode::add_fr_velodyne, this);
        front_left_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/front_left/velodyne_points", 1, &PreprocessingNode::add_fl_velodyne, this);
        rear_right_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/rear_right/velodyne_points", 1, &PreprocessingNode::add_rr_velodyne, this);
        rear_left_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/rear_left/velodyne_points", 1, &PreprocessingNode::add_rl_velodyne, this);
        top_middle_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/top_middle/velodyne_points", 1, &PreprocessingNode::add_tm_velodyne, this);
        front_livox_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/livoxfront/livox/lidar", 1, &PreprocessingNode::add_f_liv, this);
    
    }
    /**
     * @brief filter Pointcloud in rectancle shaped ROI
     * 
     * @param cloud_ptr         input cloud
     * @param cloud_ROI_ptr     output cloud as ROI
     */
    void get_ROI(   const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, 
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

    }
    /**
     * @brief Get the cloud part object
     * 
     * @param cloud_ptr         input cloud
     * @param cloud_part        output cloud that is a part of input
     * @param length            length of the part in [ m ]
     * @param deviation         distance from center of vehicle to start
     */
    void get_cloud_part(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr, 
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
        zpass2.setFilterLimits (z_max_ground + 0.01, roi_z_max);
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
        // Set maximum allowed angle between the model normal and the given axis in radians
        seg.setEpsAngle(max_angle); 
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

    void fusePointclouds(   pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr){
        // check if all Pointclouds are available without ground
        bool all_pointclouds_available;
        while (all_pointclouds_available = false){ 
            if (!front_right_velodyne_no_ground.empty() &&
                !front_left_velodyne_no_ground.empty())
                {
                    all_pointclouds_available = true;
                }
            else
            {
                all_pointclouds_available = false;
            }
        }
        // fuse pointclouds
        *no_ground_ptr = front_right_velodyne_no_ground;
        *no_ground_ptr += front_left_velodyne_no_ground;

        *ground_ptr = front_right_velodyne_ground;
        *ground_ptr += front_left_velodyne_ground;

        // clear pointclouds
        front_right_velodyne_no_ground.clear();
        front_left_velodyne_no_ground.clear();
        front_right_velodyne_ground.clear();
        front_left_velodyne_ground.clear();
    }

    void filterPointcloud(){
        // voxelgridfilter

    }
    /**
     * @brief publish clouds
     * 
     * @param no_ground_ptr cloud that is no ground
     * @param ground_ptr    cloud that represents the ground
     */
    void publishPointcloud( pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr){
        
        // pointer to pointcloud
        pcl::PointCloud<pcl::PointXYZI> no_ground_cloud;
        pcl::PointCloud<pcl::PointXYZI> ground_cloud;
        no_ground_cloud.header.frame_id = "base_footprint";
        ground_cloud.header.frame_id = "base_footprint";
        no_ground_cloud = * no_ground_ptr;
        ground_cloud = *ground_ptr;

        // publish clouds
        nogroundpub.publish(no_ground_cloud);
        groundpub.publish(ground_cloud);
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
     * @brief fron right Velodyne: transform + ground removal
     * 
     * @param input 
     */
    void add_fr_velodyne(const pcl::PointCloud<pcl::PointXYZI> input){ // ::ConstPtr& 
        
        // 1.) transform Pointcloud
        tf::Transform transform = tf::Transform(tf_fr.getRotation(), tf_fr.getOrigin());
        pcl::PointCloud<pcl::PointXYZI> fr_vel_trans;        
        pcl_ros::transformPointCloud(input, fr_vel_trans, transform);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        *cloud_ptr = fr_vel_trans;

        // 2.) filter ROI
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        get_ROI(cloud_ptr, cloud_ROI_ptr);

        // 3.) remove Ground 3 zones
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI> no_ground;
        pcl::PointCloud<pcl::PointXYZI> ground;

        // 3.1) front
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_front_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        get_cloud_part(cloud_ROI_ptr, cloud_front_ptr, vf_front_length, vf_mid_length/2);
        remove_ground(cloud_front_ptr, no_ground_ptr, ground_ptr,vf_z_min_ground_front, vf_z_max_ground_front, vf_max_angle_front);
        no_ground = *no_ground_ptr;
        ground = *ground_ptr;
        
        // 3.2) mid
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mid_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        get_cloud_part(cloud_ROI_ptr, cloud_mid_ptr, vf_mid_length, -vf_mid_length/2);
        remove_ground(cloud_mid_ptr, no_ground_ptr, ground_ptr, vf_z_min_ground_mid, vf_z_max_ground_mid, vf_max_angle_mid);
        no_ground += *no_ground_ptr;
        ground += *ground_ptr;

        // 3.3) rear
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rear_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        get_cloud_part(cloud_ROI_ptr, cloud_mid_ptr, vf_rear_length, -vf_mid_length/2);
        remove_ground(cloud_rear_ptr, no_ground_ptr, ground_ptr, vf_z_min_ground_rear, vf_z_max_ground_rear, vf_max_angle_rear);
        no_ground += *no_ground_ptr;
        ground += *ground_ptr;
               
        // 4.) finish
        front_right_velodyne_no_ground = no_ground;
        front_right_velodyne_ground = ground;
    }

    void add_fl_velodyne(const pcl::PointCloud<pcl::PointXYZI> input){ 
        
        // 1.) transform Pointcloud
        tf::Transform transform = tf::Transform(tf_fl.getRotation(), tf_fl.getOrigin());
        pcl::PointCloud<pcl::PointXYZI> fl_vel_trans;   
        pcl_ros::transformPointCloud(input, fl_vel_trans, transform);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        *cloud_ptr = fl_vel_trans;

        // 2.) filter ROI
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ROI_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        get_ROI(cloud_ptr, cloud_ROI_ptr);

        // 3.) remove Ground 3 zones
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI> no_ground;
        pcl::PointCloud<pcl::PointXYZI> ground;

        // 3.1) front
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_front_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        get_cloud_part(cloud_ROI_ptr, cloud_front_ptr, vf_front_length, vf_mid_length/2);
        remove_ground(cloud_front_ptr, no_ground_ptr, ground_ptr,vf_z_min_ground_front, vf_z_max_ground_front, vf_max_angle_front);
        no_ground = *no_ground_ptr;
        ground = *ground_ptr;
        
        // 3.2) mid
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_mid_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        get_cloud_part(cloud_ROI_ptr, cloud_mid_ptr, vf_mid_length, -vf_mid_length/2);
        remove_ground(cloud_mid_ptr, no_ground_ptr, ground_ptr, vf_z_min_ground_mid, vf_z_max_ground_mid, vf_max_angle_mid);
        no_ground += *no_ground_ptr;
        ground += *ground_ptr;

        // 3.3) rear
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_rear_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        get_cloud_part(cloud_ROI_ptr, cloud_mid_ptr, vf_rear_length, -vf_mid_length/2);
        remove_ground(cloud_rear_ptr, no_ground_ptr, ground_ptr, vf_z_min_ground_rear, vf_z_max_ground_rear, vf_max_angle_rear);
        no_ground += *no_ground_ptr;
        ground += *ground_ptr;
               
        // 4.) finish
        front_left_velodyne_no_ground = no_ground;
        front_left_velodyne_ground = ground;
    }

    void add_rr_velodyne(const pcl::PointCloud<pcl::PointXYZI> input){
        
        // 1.) transform Pointcloud
        tf::Transform transform = tf::Transform(tf_rr.getRotation(), tf_rr.getOrigin());
        pcl::PointCloud<pcl::PointXYZI> rr_vel_trans;
        pcl_ros::transformPointCloud(input, rr_vel_trans, transform);
    }

    void add_rl_velodyne(const pcl::PointCloud<pcl::PointXYZI> input){ 
        
        // 1.) transform Pointcloud
        tf::Transform transform = tf::Transform(tf_rl.getRotation(), tf_rl.getOrigin());
        pcl::PointCloud<pcl::PointXYZI> rl_vel_trans;
        pcl_ros::transformPointCloud(input, rl_vel_trans, transform);
    }

    void add_tm_velodyne(const pcl::PointCloud<pcl::PointXYZI> input){
        
        // 1.) transform Pointcloud
        tf::Transform transform = tf::Transform(tf_tm.getRotation(), tf_tm.getOrigin());
        pcl::PointCloud<pcl::PointXYZI> tm_vel_trans;
        pcl_ros::transformPointCloud(input, tm_vel_trans, transform);
    }

    void add_f_liv(const pcl::PointCloud<pcl::PointXYZI> input){ 
        
        // 1.) transform Pointcloud
        tf::Transform transform = tf::Transform(tf_f_liv.getRotation(), tf_f_liv.getOrigin());
        pcl::PointCloud<pcl::PointXYZI> f_liv_trans;
        pcl_ros::transformPointCloud(input, f_liv_trans, transform);
    }

    // Pointclouds
    pcl::PointCloud<pcl::PointXYZI> front_right_velodyne_ground;
    pcl::PointCloud<pcl::PointXYZI> front_right_velodyne_no_ground;

    pcl::PointCloud<pcl::PointXYZI> front_left_velodyne_ground;
    pcl::PointCloud<pcl::PointXYZI> front_left_velodyne_no_ground;

    pcl::PointCloud<pcl::PointXYZI> rear_right_velodyne_ground;
    pcl::PointCloud<pcl::PointXYZI> rear_right_velodyne_no_ground;

    pcl::PointCloud<pcl::PointXYZI> rear_left_velodyne_ground;
    pcl::PointCloud<pcl::PointXYZI> rear_left_velodyne_no_ground;

    pcl::PointCloud<pcl::PointXYZI> top_middle_velodyne_ground;
    pcl::PointCloud<pcl::PointXYZI> top_middle_velodyne_no_ground;

    pcl::PointCloud<pcl::PointXYZI> livox_ground;
    pcl::PointCloud<pcl::PointXYZI> livox_no_ground;

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