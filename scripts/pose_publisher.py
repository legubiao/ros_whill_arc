#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

def publish_robot_pose():
    rospy.init_node('robot_pose_publisher')
    listener = tf.TransformListener()
    pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = '/map'
        pose.pose.position.x = trans[0]
        pose.pose.position.y = trans[1]
        pose.pose.position.z = trans[2]
        pose.pose.orientation.x = rot[0]
        pose.pose.orientation.y = rot[1]
        pose.pose.orientation.z = rot[2]
        pose.pose.orientation.w = rot[3]
        pose_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_robot_pose()
    except rospy.ROSInterruptException:
        pass
