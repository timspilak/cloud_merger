#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32

def get_rotation (orientation_q):
    global roll, pitch, yaw
    rospy.loginfo("tf_received")
    orientation_list = [orientation_q[0], orientation_q[1], orientation_q[2], orientation_q[3]]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    rospy.loginfo("yaw = %s", yaw)
    rospy.loginfo("pitch = %s", pitch)
    rospy.loginfo("roll = %s", roll)
    #print yaw

def make_node_things():
    # init node
    rospy.init_node('my_quaternion_to_euler')

    # subscribers and publishers
    pub = rospy.Publisher("/my_yaw", Float32, queue_size = 1)
    listener = tf.TransformListener()

    #sub = rospy.Subscriber ('/tf_static/transforms/', Odometry, callback_get_rotation)

    # do ros things
    r = rospy.Rate(10)
    rospy.loginfo("tf node started now publishing euler angles")

    while not rospy.is_shutdown():
        try:
            (trans_0,rot_0) = listener.lookupTransform('/base_footprint', '/livox_front', rospy.Time(0))
            print(rot_0)
            (trans_1,rot_1) = listener.lookupTransform('/base_footprint', '/velodyne_front_left', rospy.Time(0))
            print(rot_1)
            (trans_2,rot_2) = listener.lookupTransform('/base_footprint', '/velodyne_front_right', rospy.Time(0))
            print(rot_2)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        get_rotation(rot_2)
        pub.publish(yaw)

        r.sleep()

if __name__ == '__main__':

    make_node_things()

# searched tf's
# /base_footprint /livox_front
# /base_footprint /velodyne_front_left
# /base_footprint /velodyne_front_right