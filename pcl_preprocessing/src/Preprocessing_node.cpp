#include "PreprocessingNode.h"

// -----------------------------------------------------------------------------------------
// Node
// -----------------------------------------------------------------------------------------
int main(int argc, char**argv)
{
    // initialize ROS
    ros::init(argc, argv, "Preprocessing_node"); // name of node
    // ros:: asyncSpinner 
    // wichtig wait for shutdown
    // create ROS-node
    PreprocessingNode node = PreprocessingNode();
    
    // Set rate to 10 hz
    ros::Rate loop_rate(10);

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

        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
        // 1.) fuse Pointclouds
        node.fusePointclouds(no_ground_ptr, ground_ptr);

        // 2.) remove outliers
        node.outlierRemoval(no_ground_ptr);

        // 3.) Voxelgrid filter
        node.voxelgrid(no_ground_ptr,voxel_cloud_ptr);

        // 4.) publish Pointcloud
        node.publishPointcloud(no_ground_ptr, ground_ptr, voxel_cloud_ptr);
 
        // do ROS things
        ros::spinOnce();
        loop_rate.sleep();
    }

}
