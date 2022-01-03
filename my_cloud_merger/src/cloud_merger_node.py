#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2

# ------ beginning of input_cloud class ------
class input_cloud():
    def __init__(self, cloud_name, cloud_transformation, cloud_orientation_q):
        self.name = cloud_name
        self.transformation = cloud_transformation
        self.orientation_q = cloud_orientation_q

    def get_rotation (orientation_q):
        global roll, pitch, yaw
        rospy.loginfo("tf_received" + self.name)
        orientation_list = [orientation_q[0], orientation_q[1], orientation_q[2], orientation_q[3]]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        rospy.loginfo("yaw = %s", yaw)
        rospy.loginfo("pitch = %s", pitch)
        rospy.loginfo("roll = %s", roll)
        #print yaw

# ------ end of input_cloud class ------
        
# ------ beginning of ouptut_cloud class ------        
class ouptut_cloud():
    # probably unused
    def __init__(self, cloud_name, cloud_transformation, cloud_orientation_q):
        self.name = cloud_name
        self.transformation = cloud_transformation
        self.orientation_q = cloud_orientation_q
        input_clouds_list = [] 

        # init node
        rospy.init_node('my_cloud_merger')

        # listener
        listener = tf.TransformListener()

        # do ros things
        r = rospy.Rate(10)
        rospy.loginfo("tf node started")
        appended_all_clouds = False

        while not rospy.is_shutdown() and not appended_all_clouds:
            try:
                # inputcloud 1
                (trans_1,rot_1) = listener.lookupTransform('/base_footprint', '/livox_front', rospy.Time(0))
                input_cloud1 = input_cloud("livox_front", trans_1, rot_1)
                input_clouds_list.append(input_cloud1)
                rospy.loginfo("appended cloud1 successfully")
                # inputcloud 2
                (trans_2,rot_2) = listener.lookupTransform('/base_footprint', '/velodyne_front_left', rospy.Time(0))
                input_cloud2 = input_cloud("velodyne_front_left", trans_2, rot_2)
                input_clouds_list.append(input_cloud2)
                rospy.loginfo("appended cloud2 successfully")
                # inputcloud 3
                (trans_3,rot_3) = listener.lookupTransform('/base_footprint', '/velodyne_front_right', rospy.Time(0))
                input_cloud3 = input_cloud("velodyne_front_right", trans_3, rot_3)
                input_clouds_list.append(input_cloud3)
                rospy.loginfo("appended cloud3 successfully")

                appended_all_clouds = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        rospy.loginfo("initialized inputclouds trans and rot")
        r.sleep()

    def merge_clouds():
        # publisher
        ouput_publisher = rospy.Publisher("/points_merged", PointCloud2, queue_size = 1)
        # while loop that constatly merges and buffers inputclouds
        # then publishing the outputcloud
# ----- end of ouptut_cloud class ------

if __name__ == '__main__':

    output_cloud = ouptut_cloud("/points_raw","", "") 
    output_cloud.merge_clouds()

# searched tf's
# /base_footprint /livox_front
# /base_footprint /velodyne_front_left
# /base_footprint /velodyne_front_right