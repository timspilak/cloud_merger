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
        front_right_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/front_right/velodyne_points", 1, &CloudFusionNode::add_fr_velodyne, this);
        front_left_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/front_left/velodyne_points", 1, &CloudFusionNode::add_fl_velodyne, this);
        front_livox_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/livoxfront/livox/lidar", 1, &CloudFusionNode::add_liv, this);
    }

    // cloud transformation
    void cloud_transformation(){
        /**    try to use with return
        tf::Transformation transformation = tf::Transform(transform.getRotation(), transform.getOrigin());
        pcl_ros::transformPointCloud(cloud, cloud, transform);
        return cloud;**/
        
        // transform front right velodyne
        tf::Transform transform = tf::Transform(tf_fr.getRotation(), tf_fr.getOrigin());
        pcl_ros::transformPointCloud(fr_vel_raw, fr_vel_trans, transform);

        // transform front left velodyne
        transform = tf::Transform(tf_fl.getRotation(), tf_fl.getOrigin());
        pcl_ros::transformPointCloud(fl_vel_raw, fl_vel_trans, transform);
        
        // transform front left velodyne
        transform = tf::Transform(tf_liv.getRotation(), tf_liv.getOrigin());
        pcl_ros::transformPointCloud(liv_raw, liv_trans, transform);
    }

    // cloud fusion
    void cloud_fusion(){
        fused_cloud = fr_vel_trans;
        fused_cloud += fl_vel_trans;
        //fused_cloud = liv_trans;
        liv_trans.clear();
    }

    // remove outliers
    void remove_outliers(){
        //pcl::toROSMsg(fused_cloud, output_cloud);
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> filter;
        // configurate the filter
        filter.setInputCloud(fused_cloud&);
        filter.setRadiusSearch(0.3);
        filter.setMinNeighborsInRadius (2);
        //filter.setKeepOrganized(true);
        // apply filter
        filter.filter(*cloud_filtered);
    }


    void publish_cloud(){
        // set default values of outputcloud
        fused_cloud.header.frame_id = "base_footprint";

        // publish fused points
        fusionpub.publish(fused_cloud);

        // clear outputcloud
        fused_cloud.clear();
    }

    // Transforms
    tf::StampedTransform tf_fr;
    tf::StampedTransform tf_fl;
    tf::StampedTransform tf_liv;

private:
        /** debugging 
        debugmsg.data = "received Message from " + input.header.frame_id; 
        ROS_INFO_STREAM(debugmsg);**/

    // Callback function velodyne+ livox       
    void add_fr_velodyne(const pcl::PointCloud<pcl::PointXYZI> input){ // eigentlich ::ConstPtr& Verstehe ich nicht
        fr_vel_raw = input;
    }
    void add_fl_velodyne(const pcl::PointCloud<pcl::PointXYZI> input){ 
        fl_vel_raw = input;
    }
     
    void add_liv(const pcl::PointCloud<pcl::PointXYZI> input){ 
        liv_raw += input;
    }

    // Pointclouds
    //pcl::PointCloud<pcl::PointXYZ>::Ptr fr_vel_raw (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZI> fr_vel_raw;
    pcl::PointCloud<pcl::PointXYZI> fl_vel_raw;
    pcl::PointCloud<pcl::PointXYZI> liv_raw;


    pcl::PointCloud<pcl::PointXYZI> fr_vel_trans;
    pcl::PointCloud<pcl::PointXYZI> fl_vel_trans;
    pcl::PointCloud<pcl::PointXYZI> liv_trans;

    pcl::PointCloud<pcl::PointXYZI> fused_cloud;
    
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fused (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    //pcl::PointCloud<pcl::PointXYZ> output_cloud;
    sensor_msgs::PointCloud2 output_cloud;

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
        // get all Transformations
        try{
            listener_front_right.lookupTransform("/base_footprint","/velodyne_front_right", ros::Time(0), node.tf_fr);
            listener_front_left.lookupTransform("/base_footprint","/velodyne_front_left", ros::Time(0), node.tf_fl);
            listener_livox.lookupTransform("/base_footprint","/livox_front", ros::Time(0), node.tf_liv);
        }catch(tf::TransformException ex){
            ROS_ERROR("%s", ex.what());
        }

        // transform all input clouds
        node.cloud_transformation();

        // fuse all input clouds
        node.cloud_fusion();

        // remove outliers
        node.remove_outliers();

        // remove > 4m

        // remove out of Range

        // remove ground

        // publish final cloud
        node.publish_cloud();

        // do ROS things
        ros::spinOnce();
        loop_rate.sleep();
    }

}