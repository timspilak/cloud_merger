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
        //node.cloud_fusion(cloud_ptr);
        node.get_cloud(cloud_ptr);
        
        // 2.) filter ROI
        //node.filter_ROI_T(cloud_ptr, filtered_cloud_ptr);
       
        // 2.a) filter ROI rectangle
        pcl::PointCloud<pcl::PointXYZI>::Ptr front_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr mid_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr rear_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        node.filter_ROI_R(cloud_ptr,front_cloud_ptr, mid_cloud_ptr, rear_cloud_ptr);

        // 3.) remove outliers
        //node.remove_outliers(filtered_cloud_ptr);
        node.remove_outliers(front_cloud_ptr);
        node.remove_outliers(mid_cloud_ptr);
        node.remove_outliers(rear_cloud_ptr);

        // 4.) remove ground
/**        node.remove_ground(filtered_cloud_ptr, no_ground_cloud_ptr, ground_cloud_ptr);
        pcl::PointCloud<pcl::PointXYZI> ground1;
        ground1 = *ground_cloud_ptr;
        node.remove_ground(no_ground_cloud_ptr, no_ground_cloud_ptr, ground_cloud_ptr);
        pcl::PointCloud<pcl::PointXYZI> ground2;
        ground2 = *ground_cloud_ptr;
        ground1 += ground2;
        *ground_cloud_ptr = ground1;
**/
        // remove ground front
        node.remove_ground(front_cloud_ptr, no_ground_cloud_ptr, ground_cloud_ptr, z_min_ground_front, z_max_ground_front, max_angle_front);
        pcl::PointCloud<pcl::PointXYZI> ground1;
        ground1 = *ground_cloud_ptr;
        pcl::PointCloud<pcl::PointXYZI> noground1;
        noground1 = *no_ground_cloud_ptr;

        // remove ground mid
        node.remove_ground(mid_cloud_ptr, no_ground_cloud_ptr, ground_cloud_ptr, z_min_ground_mid, z_max_ground_mid, max_angle_mid);
        pcl::PointCloud<pcl::PointXYZI> ground2;
        ground2 = *ground_cloud_ptr;
        pcl::PointCloud<pcl::PointXYZI> noground2;
        noground2 = *no_ground_cloud_ptr;
        
        // remove ground rear
        node.remove_ground(rear_cloud_ptr, no_ground_cloud_ptr, ground_cloud_ptr, z_min_ground_rear, z_max_ground_rear, max_angle_rear);
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

        // 5.) voxelgrid
        node.voxelgrid(no_ground_cloud_ptr, voxel_cloud_ptr);        

        // 6.) publish final cloud
        node.publish(cloud_ptr, filtered_cloud_ptr, no_ground_cloud_ptr, ground_cloud_ptr, voxel_cloud_ptr); // publisher uebergeben

        // do ROS things
        ros::spinOnce();
        loop_rate.sleep();
    }

}
