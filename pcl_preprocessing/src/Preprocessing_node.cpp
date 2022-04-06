#include "PreprocessingNode.h"

// -----------------------------------------------------------------------------------------
// Node
// -----------------------------------------------------------------------------------------
int main(int argc, char**argv)
{
    // initialize ROS
    ros::init(argc, argv, "Preprocessing_node"); // name of node
    
    // create ROS-node
    PreprocessingNode node = PreprocessingNode();
    
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

        // 1.) fuse Pointclouds
        node.fusePointclouds();

        // 2.) filter Pointcloud
        node.filterPointcloud();

        // 3.) publish Pointcloud
        node.publishPointcloud();

        
        // do ROS things
        ros::spinOnce();
        loop_rate.sleep();
    }

}
