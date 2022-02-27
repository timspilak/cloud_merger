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
// filter areas
#include <pcl/filters/passthrough.h>
// remove ground
/**
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
**/
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

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
        fusionpub = node.advertise<pcl::PointCloud<pcl::PointXYZI>>("/cloud_fusion_node/points_filtered", 1); // topic name, queue <sensor_msgs::PointCloud2>
        comparepub = node.advertise<pcl::PointCloud<pcl::PointXYZI>>("/cloud_fusion_node/points_fused", 1); 
        comparepub_1 = node.advertise<pcl::PointCloud<pcl::PointXYZI>>("/cloud_fusion_node/points_cut", 1); 

        // subscriber
        front_right_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/front_right/velodyne_points", 1, &CloudFusionNode::add_fr_velodyne, this);
        front_left_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/front_left/velodyne_points", 1, &CloudFusionNode::add_fl_velodyne, this);
        rear_right_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/rear_right/velodyne_points", 1, &CloudFusionNode::add_rr_velodyne, this);
        rear_left_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/rear_left/velodyne_points", 1, &CloudFusionNode::add_rl_velodyne, this);
        top_middle_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/velodyne/top_middle/velodyne_points", 1, &CloudFusionNode::add_tm_velodyne, this);
        front_livox_sub = node.subscribe<pcl::PointCloud<pcl::PointXYZI>>("/livoxfront/livox/lidar", 1, &CloudFusionNode::add_f_liv, this);
    }

    // cloud fusion
    // benötigt die clouds roh
    pcl::PointCloud<pcl::PointXYZI> cloud_fusion(pcl::PointCloud<pcl::PointXYZI> output_cloud){

        output_cloud = fr_vel_trans;
        output_cloud += fl_vel_trans;
        output_cloud += rr_vel_trans;
        output_cloud += rl_vel_trans;
        output_cloud += tm_vel_trans;
        output_cloud += f_liv_trans;

        f_liv_trans.clear();
        return output_cloud;
    }

    // remove outliers
    pcl::PointCloud<pcl::PointXYZI> remove_outliers(pcl::PointCloud<pcl::PointXYZI>::Ptr fused_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud){
        
        // build the filter
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
        outrem.setInputCloud(fused_cloud);
        outrem.setRadiusSearch(0.3);
        outrem.setMinNeighborsInRadius(1);
        outrem.setKeepOrganized(false);

        // apply filter
        outrem.filter (*filtered_cloud);

        // return cloud
        pcl::PointCloud<pcl::PointXYZI> output;
        output = *filtered_cloud;
        return output;
    }

    // filter in axis
    pcl::PointCloud<pcl::PointXYZI> filter_axis(pcl::PointCloud<pcl::PointXYZI>::Ptr fused_cloud, 
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud, double lower, double upper, std::string axis){

        // build the filter
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud (fused_cloud);
        pass.setFilterFieldName (axis);
        pass.setFilterLimits (lower, upper);
        //pass.setFilterLimitsNegative (true);
       
        // apply filter
        pass.filter (*filtered_cloud);

        // return cloud
        pcl::PointCloud<pcl::PointXYZI> output;
        output = *filtered_cloud;
        return output;
    }

    // remove ground with RANSAC pcl::PointCloud<pcl::PointXYZI>
    void remove_ground(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
                                                    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud,
                                                    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud){

        // preperations
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZI> seg;

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        //seg.setMaxIterations (1000);
        seg.setDistanceThreshold (0.01);
        seg.setInputCloud (cloud);
        seg.setProbability(1.0);
        seg.segment (*inliers, *coefficients);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZI> extract;

        // Extract the inliers
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (*no_ground_cloud);

        extract.setNegative(false);
        extract.filter(*ground_cloud);
/**
        // preparations
        pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_plane (new pcl::SampleConsensusModelPlane<pcl::PointXYZI> (cloud));
        std::vector<int> ground;
        //pcl::PointIndices::Ptr ground (new pcl::PointIndices ());

        // RANSAC
        pcl::RandomSampleConsensus<pcl::PointXYZI> ransac (model_plane);
        ransac.setDistanceThreshold(0.05);
        ransac.computeModel();
        ransac.getInliers(ground);

        // copies all inliers of the model computed to another PointCloud
        pcl::copyPointCloud (*cloud, ground, *filtered_cloud);
/**        
        // extract ground
        extract.setInputCloud(cloud);
        extract.setIndices(ground);
        extract.setNegative(true);
        extract.filter(*filtered_cloud);
**/
        // return cloud
        //pcl::PointCloud<pcl::PointXYZI> output;
        //output = *filtered_cloud;
        //return output;
    }

    void publish_cloud(pcl::PointCloud<pcl::PointXYZI> output_cloud, pcl::PointCloud<pcl::PointXYZI> filtered_cloud, pcl::PointCloud<pcl::PointXYZI> cut_cloud){
        // set default values of outputcloud
        output_cloud.header.frame_id = "base_footprint";
        //pcl_conversions::toPCL(ros::Time::now(), output_cloud.header.stamp);
        
        // publish fused points
        fusionpub.publish(output_cloud);

        // for comaprison
        filtered_cloud.header.frame_id = "base_footprint";
        comparepub.publish(filtered_cloud);

        cut_cloud.header.frame_id = "base_footprint";
        comparepub_1.publish(cut_cloud);
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
    ros::Publisher comparepub;
    ros::Publisher comparepub_1;

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
    ros::Rate loop_rate(1);

    // get transformations  
    tf::TransformListener listener_fr;
    tf::TransformListener listener_fl;
    tf::TransformListener listener_rr;
    tf::TransformListener listener_rl;
    tf::TransformListener listener_tm;
    tf::TransformListener listener_f_liv;
    bool got_transformations = false;

    
    while (ros::ok())
    {
        if (!got_transformations){
            // get all Transformations
            try{
                listener_fr.lookupTransform("/base_footprint","/velodyne_front_right", ros::Time(0), node.tf_fr);
                listener_fl.lookupTransform("/base_footprint","/velodyne_front_left", ros::Time(0), node.tf_fl);
                listener_rr.lookupTransform("/base_footprint","/velodyne_rear_right", ros::Time(0), node.tf_rr);
                listener_rl.lookupTransform("/base_footprint","/velodyne_rear_left", ros::Time(0), node.tf_rl);
                listener_tm.lookupTransform("/base_footprint", "/velodyne_top_middle", ros::Time(0), node.tf_tm);
                listener_f_liv.lookupTransform("/base_footprint","/livox_front", ros::Time(0), node.tf_f_liv);
                got_transformations = true;
            }catch(tf::TransformException ex){
                ROS_ERROR("%s", ex.what());
            }
        }
        
        // Clouds
        pcl::PointCloud<pcl::PointXYZI> output_cloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);

        // for comaprison
        pcl::PointCloud<pcl::PointXYZI> fused_cloud;
        pcl::PointCloud<pcl::PointXYZI> ground_cloud;

        // fuse all input clouds
        output_cloud = node.cloud_fusion(output_cloud);
        fused_cloud = output_cloud;
        *cloud = output_cloud;

        // filter ROI
        *cloud = node.filter_axis(cloud, filtered_cloud, -0.5, 2.0, "z");        // height
        *cloud = node.filter_axis(cloud, filtered_cloud, -20.0, 20.0, "x");     // length
        *cloud = node.filter_axis(cloud, filtered_cloud, -5.0, 5.0, "y");       // length

        // remove outliers
        *cloud = node.remove_outliers(cloud, filtered_cloud);

        // remove ground
        node.remove_ground(cloud, filtered_cloud,ground_cloud_ptr);
        ground_cloud = *ground_cloud_ptr;
        // publish final cloud
        output_cloud = *cloud;
        node.publish_cloud(output_cloud, fused_cloud, ground_cloud);

        // clear outputcloud
        output_cloud.clear();

        // do ROS things
        ros::spinOnce();
        loop_rate.sleep();
    }

}

// pointer to cloud
//      output = *filtered_cloud
// cloud to pointer
//      *cloud = output_cloud;