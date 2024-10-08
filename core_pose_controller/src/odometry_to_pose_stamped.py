#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def odom_callback(msg):
    pose_stamped = PoseStamped()
    pose_stamped.header = msg.header
    pose_stamped.pose = msg.pose.pose

    pose_stamped_pub.publish(pose_stamped)

if __name__ == '__main__':
    rospy.init_node('odom_to_pose')

    odom_sub = rospy.Subscriber('odometry', Odometry, odom_callback)

    pose_stamped_pub = rospy.Publisher('pose_stamped', PoseStamped, queue_size=1)

    rospy.spin()
