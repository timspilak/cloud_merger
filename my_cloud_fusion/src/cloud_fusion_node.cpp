#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>

#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <pcl_ros/point_cloud.h>

class CloudFusionNode
{
public:
    // constructor

    CloudFusionNode(){
        ROS_INFO("Fusion node is now running");
        // publisher
        fusionpub = node.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/cloud_fusion_node/points_fused", 1); // topic name, queue <sensor_msgs::PointCloud2>

        // subscriber
        front_right_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>("/velodyne/front_right/velodyne_points", 1, &CloudFusionNode::add_cloud_velodyne, this);
        front_left_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>("/velodyne/front_left/velodyne_points", 1, &CloudFusionNode::add_cloud_velodyne, this);
        front_livox_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>("/livoxfront/livox/lidar", 1, &CloudFusionNode::add_cloud_velodyne, this);
    }

    // cloud transformation
    void cloud_transformation(){
       pcl::transformPointCloud(fr_vel_raw,fr_vel_raw, tf_fr, true);
       //pcl::transformPointCloud(fl_vel_raw,fl_vel_raw, tf_fl, true);
       //pcl::transformPointCloud(liv_raw,liv_raw, tf_liv, true);
    }

    // cloud fusion
    void cloud_fusion(){
        output_cloud = fr_vel_raw;
        output_cloud += fl_vel_raw;
        //output_cloud += liv_raw;

        // set default values of outputcloud
        output_cloud.header.frame_id = "points_fused";

        fusionpub.publish(output_cloud);
    }

    // Transforms
    tf::StampedTransform tf_fr;
    tf::StampedTransform tf_fl;
    tf::StampedTransform tf_liv;

private:
    // Callback function velodyne+ livox       
    void add_cloud_velodyne(const pcl::PointCloud<pcl::PointXYZRGB> input){ // eigentlich ::ConstPtr& Verstehe ich nicht
        /** debugging 
        debugmsg.data = "received Message from " + input.header.frame_id; 
        ROS_INFO_STREAM(debugmsg);**/
        if (input.header.frame_id == "velodyne_front_right")
        {
            fr_vel_raw = input;
        }
        else if (input.header.frame_id == "velodyne_front_left")
        {
            fl_vel_raw = input;
        }
        else if (input.header.frame_id == "livox_front ")
        {
            liv_raw = input;
        }
        

    }

    // Pointclouds
    pcl::PointCloud<pcl::PointXYZRGB> fr_vel_raw;
    pcl::PointCloud<pcl::PointXYZRGB> fl_vel_raw;
    pcl::PointCloud<pcl::PointXYZRGB> liv_raw;

    pcl::PointCloud<pcl::PointXYZRGB> output_cloud;

    // ROS node
    ros::NodeHandle node{ "~"};

    // subscriber
    ros:: Subscriber front_right_sub;
    ros:: Subscriber front_left_sub;
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
    tf::TransformListener listener_front_right;
    tf::TransformListener listener_front_left;
    tf::TransformListener listener_livox;

    
        while (ros::ok())
    {
        /****/
        try{
            listener_front_right.lookupTransform("/base_footprint","/velodyne_front_right", ros::Time(0), node.tf_fr);
            listener_front_right.lookupTransform("/base_footprint","/velodyne_front_left", ros::Time(0), node.tf_fl);
            listener_front_right.lookupTransform("/base_footprint","/livox_front", ros::Time(0), node.tf_liv);
        }catch(tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
        }

        // transform all input clouds
        node.cloud_transformation();

        // fuse all input clouds
        node.cloud_fusion();

        // do ROS things
        ros::spinOnce();
        loop_rate.sleep();
    }

}