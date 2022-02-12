#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
// filter outliers
#include <pcl/filters/radius_outlier_removal.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>


class CloudFusionNode
{
public:
    // constructor
    CloudFusionNode(){
        // Debugging
        ROS_INFO("Fusion node is now running");

        // publisher
        fusionpub = node.advertise<pcl::PointCloud<pcl::PointXYZI>>("/cloud_fusion_node/points_fused", 1); // topic name, queue <sensor_msgs::PointCloud2>
        
        // subscriber
        front_right_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/velodyne/front_right/velodyne_points", 1, &CloudFusionNode::add_fr_velodyne, this);
        front_left_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/velodyne/front_left/velodyne_points", 1, &CloudFusionNode::add_fl_velodyne, this);
        rear_right_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/velodyne/rear_right/velodyne_points", 1, &CloudFusionNode::add_rr_velodyne, this);
        rear_left_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/velodyne/rear_left/velodyne_points", 1, &CloudFusionNode::add_rl_velodyne, this);
        top_middle_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/velodyne/top_middle/velodyne_points", 1, &CloudFusionNode::add_tm_velodyne, this);
        front_livox_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/livoxfront/livox/lidar", 1, &CloudFusionNode::add_f_liv, this);
    }

    // cloud fusion
    pcl::PointCloud<pcl::PointXYZ> cloud_fusion(pcl::PointCloud<pcl::PointXYZ> output_cloud){

        output_cloud = fr_vel_trans;
        output_cloud += fl_vel_trans;
        output_cloud += rr_vel_trans;
        output_cloud += rl_vel_trans;
        output_cloud += tm_vel_trans;
        output_cloud += f_liv_trans;

        f_liv_trans.clear();
        return output_cloud;
    }


    pcl::PointCloud<pcl::PointXYZ> remove_outliers(pcl::PointCloud<pcl::PointXYZ>::Ptr fused_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud){
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        // build the filter
        outrem.setInputCloud(fused_cloud);
        outrem.setRadiusSearch(0.3);
        outrem.setMinNeighborsInRadius (3);
        outrem.setKeepOrganized(true);
        // apply filter
        outrem.filter (*filtered_cloud);
        pcl::PointCloud<pcl::PointXYZ> output;
        output = *filtered_cloud;
        return output;
    }


    void publish_cloud(pcl::PointCloud<pcl::PointXYZ> fused_cloud){
        // set default values of outputcloud
        fused_cloud.header.frame_id = "base_footprint";
        //pcl_conversions::toPCL(ros::Time::now(), fused_cloud.header.stamp);
        
        // publish fused points
        fusionpub.publish(fused_cloud);
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

    void add_fr_velodyne(const pcl::PointCloud<pcl::PointXYZ> input){ // ::ConstPtr& 
        tf::Transform transform = tf::Transform(tf_fr.getRotation(), tf_fr.getOrigin());
        pcl_ros::transformPointCloud(input, fr_vel_trans, transform);
    }

    void add_fl_velodyne(const pcl::PointCloud<pcl::PointXYZ> input){ 
        tf::Transform transform = tf::Transform(tf_fl.getRotation(), tf_fl.getOrigin());
        pcl_ros::transformPointCloud(input, fl_vel_trans, transform);
    }

    void add_rr_velodyne(const pcl::PointCloud<pcl::PointXYZ> input){
        tf::Transform transform = tf::Transform(tf_rr.getRotation(), tf_rr.getOrigin());
        pcl_ros::transformPointCloud(input, rr_vel_trans, transform);
    }

    void add_rl_velodyne(const pcl::PointCloud<pcl::PointXYZ> input){ 
        tf::Transform transform = tf::Transform(tf_rl.getRotation(), tf_rl.getOrigin());
        pcl_ros::transformPointCloud(input, rl_vel_trans, transform);
    }

    void add_tm_velodyne(const pcl::PointCloud<pcl::PointXYZ> input){
        tf::Transform transform = tf::Transform(tf_tm.getRotation(), tf_tm.getOrigin());
        pcl_ros::transformPointCloud(input, tm_vel_trans, transform);
    }

    void add_f_liv(const pcl::PointCloud<pcl::PointXYZ> input){ 
        tf::Transform transform = tf::Transform(tf_f_liv.getRotation(), tf_f_liv.getOrigin());
        pcl_ros::transformPointCloud(input, f_liv_trans, transform);
    }


    // Pointclouds
    pcl::PointCloud<pcl::PointXYZ> fr_vel_trans;
    pcl::PointCloud<pcl::PointXYZ> fl_vel_trans;
    pcl::PointCloud<pcl::PointXYZ> rr_vel_trans;
    pcl::PointCloud<pcl::PointXYZ> rl_vel_trans;
    pcl::PointCloud<pcl::PointXYZ> tm_vel_trans;
    pcl::PointCloud<pcl::PointXYZ> f_liv_trans;

    // ROS node
    ros::NodeHandle node{ "~"};

    // subscriber
    ros:: Subscriber front_right_sub;
    ros:: Subscriber front_left_sub;
    ros:: Subscriber rear_right_sub;
    ros:: Subscriber rear_left_sub;
    ros:: Subscriber top_middle_sub;
    ros:: Subscriber front_livox_sub;

    // publisher
    ros::Publisher fusionpub;

    // create msgs
    std_msgs::String debugmsg;

};

// Main

int main(int argc, char**argv)
{
    // initialize ROS
    ros::init(argc, argv, "cloud_fusion_node"); // name of node
    
    // create ROS-node
    CloudFusionNode node = CloudFusionNode();
    
    // Set rate to 10 hz
    ros::Rate loop_rate(10);

    // get transformations  
    tf::TransformListener listener_fr;
    tf::TransformListener listener_fl;
    tf::TransformListener listener_rr;
    tf::TransformListener listener_rl;
    tf::TransformListener listener_tm;
    tf::TransformListener listener_f_liv;
    

    
    while (ros::ok())
    {
        // get all Transformations
        try{
            listener_fr.lookupTransform("/base_footprint","/velodyne_front_right", ros::Time(0), node.tf_fr);
            listener_fl.lookupTransform("/base_footprint","/velodyne_front_left", ros::Time(0), node.tf_fl);
            listener_rr.lookupTransform("/base_footprint","/velodyne_rear_right", ros::Time(0), node.tf_rr);
            listener_rl.lookupTransform("/base_footprint","/velodyne_rear_left", ros::Time(0), node.tf_rl);
            listener_tm.lookupTransform("/base_footprint", "/velodyne_top_middle", ros::Time(0), node.tf_tm);
            listener_f_liv.lookupTransform("/base_footprint","/livox_front", ros::Time(0), node.tf_f_liv);
        }catch(tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
        }
        // Clouds
        pcl::PointCloud<pcl::PointXYZ> output_cloud;

        // fuse all input clouds
        output_cloud = node.cloud_fusion(output_cloud);

        // remove outliers
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        *cloud = output_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        output_cloud = node.remove_outliers(cloud, filtered_cloud);
        // remove > 4m
        //output_cloud = node.trim_cloud(output_cloud);
        // remove out of Range

        // remove ground

        // publish final cloud
        node.publish_cloud(output_cloud);

        // clear outputcloud
        output_cloud.clear();

        // do ROS things
        ros::spinOnce();
        loop_rate.sleep();
    }

}