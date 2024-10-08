#!/usr/bin/env python

import rospy
from rospy import Header
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge
from message_filters import TimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry 
import torch

from core_trajectory_msgs.msg import FixedTrajectory
from diagnostic_msgs.msg import KeyValue

class VisualServoUtils:
    def __init__(self):
        # Load the YOLO model
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/mohammad/workspace/16-667/src/visual_servo_landing/model/yolov5_landing_pad/best.pt')
        # Initialize ROS publishers
        self.velocity_pub = rospy.Publisher("velocity_setpoint", TwistStamped, queue_size=10)
        self.img_debug_pub = rospy.Publisher("image_debug", Image, queue_size=2)
        self.pose_ctrl_mute_pub = rospy.Publisher("pose_controller/mute_control", Bool, queue_size=100)
        self.fixed_traj_pub = rospy.Publisher("/fixed_trajectory", FixedTrajectory, queue_size=100)
 
        self.odom = Odometry()       
    def publish_debug_image(self, image):
        header = Header(stamp=rospy.Time.now())
        img_processed_msg = Image()
        img_processed_msg.data = image.tobytes()
        img_processed_msg.encoding = 'rgb8'
        img_processed_msg.header = header
        img_processed_msg.height = image.shape[0]
        img_processed_msg.width = image.shape[1]                
        img_processed_msg.step = image.shape[1] * image.shape[2]
        self.img_debug_pub.publish(img_processed_msg)

    def update_odometry(self, odom):
        self.odom = odom

    def detect_landing_pad(self, image, depth):
        center_x, center_y, detected_z = None, None, None
        if self.model is not None:
            result = self.model(image)
            debug_image = image.copy()
            detections = result.xyxy[0]
            for x1, y1, x2, y2, conf, cls in detections:
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                self.publish_debug_image(debug_image)

            detected_z = depth[int(center_x), int(center_y)]
        
        return center_x, center_y, detected_z

    def publish_velocity_ibvs(self, vx, vy, vz):
        
        vel_msg = TwistStamped()
        vel_msg.header.frame_id = "/base_link_stabilized"
        vel_msg.header.stamp = rospy.get_rostime()
        vel_msg.twist.linear.x = -vy
        vel_msg.twist.linear.y = -vx
        vel_msg.twist.linear.z = vz

        msg = Bool()
        msg.data = True
        self.pose_ctrl_mute_pub.publish(msg)
        self.velocity_pub.publish(vel_msg)
        rospy.loginfo("velocity update: vx = " + str(vx) + ", vy = " + str(vy))

    def publish_position_pbvs(self, px, py, pz):
        print("publishing position")
        x = self.odom.pose.pose.position.x
        y = self.odom.pose.pose.position.y
        z = self.odom.pose.pose.position.z
        tracking_point_mag = 0.5
        mag = np.sqrt((x - px)**2 + (y - py)**2 + (z - pz)**2)
        if mag > tracking_point_mag:
            tracking_point_x = x + (px - x)/mag * tracking_point_mag
            tracking_point_y = y + (py - y)/mag * tracking_point_mag
            tracking_point_z = z + (pz - z)/mag * tracking_point_mag/4
        else:
            tracking_point_x = px
            tracking_point_y = py
            tracking_point_z = pz

        traj = FixedTrajectory()
        traj.type = "Point"
        att1 = KeyValue()
        att1.key = "frame_id"
        att1.value = "world"
        traj.attributes.append(att1)
        att2 = KeyValue()
        att2.key = "height"
        att2.value = str(tracking_point_z)
        traj.attributes.append(att2)
        att3 = KeyValue()
        att3.key = "max_acceleration"
        att3.value = str(0.4)
        traj.attributes.append(att3)
        att4 = KeyValue()
        att4.key = "velocity"
        att4.value = str(0.1)
        traj.attributes.append(att4)
        att5 = KeyValue()
        att5.key = "x"
        att5.value = str(tracking_point_x)
        traj.attributes.append(att5)
        att6 = KeyValue()
        att6.key = "y"
        att6.value = str(tracking_point_y)

        self.fixed_traj_pub.publish(traj)