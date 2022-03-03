#include "CloudFusionNode.h"

// -----------------------------------------------------------------------------------------
// Node
// -----------------------------------------------------------------------------------------
int main(int argc, char**argv)
{
    // initialize ROS
    ros::init(argc, argv, "cloud_fusion_node"); // name of node
    
    // create ROS-node
    CloudFusionNode node = CloudFusionNode();
    
    // Set rate to 10 hz
    ros::Rate loop_rate(0.5);

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
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);

        // 1.) fuse all input clouds
        node.cloud_fusion(cloud_ptr);

        // 2.) filter ROI
        node.filter_ROI(cloud_ptr, filtered_cloud_ptr);

        // 3.) remove outliers
        node.remove_outliers(filtered_cloud_ptr);

        // 4.) remove ground
        node.remove_ground(filtered_cloud_ptr, no_ground_cloud_ptr, ground_cloud_ptr);
        pcl::PointCloud<pcl::PointXYZI> ground1;
        ground1 = *ground_cloud_ptr;
        node.remove_ground(no_ground_cloud_ptr, no_ground_cloud_ptr, ground_cloud_ptr);
        pcl::PointCloud<pcl::PointXYZI> ground2;
        ground2 = *ground_cloud_ptr;
        ground1 += ground2;
        *ground_cloud_ptr = ground1;

        // 5.) voxelgrid
        node.voxelgrid(no_ground_cloud_ptr, voxel_cloud_ptr);        

        // 6.) publish final cloud
        node.publish(cloud_ptr, filtered_cloud_ptr, no_ground_cloud_ptr, ground_cloud_ptr, voxel_cloud_ptr); // publisher uebergeben

        // do ROS things
        ros::spinOnce();
        loop_rate.sleep();
    }

}
