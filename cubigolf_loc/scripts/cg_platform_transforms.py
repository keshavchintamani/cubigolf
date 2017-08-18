
"""
Subscribes to /cubigolf/odom and publishes map-odom and odom-base_link transforms.

We assume the base_link is moving in the odometry frame.
"""

import rospy
import tf
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg
import std_msgs.msg

__author__ = "Keshav Chintamani"
__copyright__ = "Copyright 2017, The Cubigolf Project"
__credits__ = ["Keshav Chintamani", "Jacob Carballido"]
__license__ = "GPL"
__version__ = "0.0.1"
__maintainer__ = "Keshav Chintamani"
__email__ = "keshav.chintamani@gmail.com"
__status__ = "devel"

def handle_odom(odom):
    br = tf2_ros.TransformBroadcaster()
    odom_tr = geometry_msgs.msg.TransformStamped()
    odom_tr.header.stamp = rospy.Time.now()
    odom_tr.header.frame_id = "map"
    odom_tr.child_frame_id = "odom"
    # Assumes the odometry is measured from mid point between the two tracks
    odom_tr.transform.translation.x = odom_tr.transform.translation.z = \
        odom_tr.transform.translation.y = 0
    odom_tr.transform.rotation.x = odom_tr.transform.rotation.z = \
        odom_tr.transform.rotation.y = 0
    odom_tr.transform.rotation.w = 1
    br.sendTransform(odom_tr)

    baselink_tr = geometry_msgs.msg.TransformStamped()
    baselink_tr.header.stamp = rospy.Time.now()
    baselink_tr.header.frame_id = "odom"
    baselink_tr.child_frame_id = "base_link"

    baselink_tr.transform.translation = odom.pose.pose.position
    baselink_tr.transform.rotation = odom.pose.pose.orientation

    br.sendTransform(baselink_tr)

if __name__ == '__main__':
    rospy.init_node('trinibot_tf_publisher', anonymous=True)
    rate = rospy.Rate(100)
    rospy.Subscriber('/cubigolf/odom',
                     nav_msgs.msg.Odometry,
                     handle_odom)

    #TODO: set parameter server with required loop rate
    rospy.spin()