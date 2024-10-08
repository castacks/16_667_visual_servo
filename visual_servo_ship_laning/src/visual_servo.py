#!/usr/bin/env python

import rospy
import rospkg
import os
from rospy import Header
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist, TwistStamped
from cv_bridge import CvBridge
from message_filters import TimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry 
import torch

from core_trajectory_msgs.msg import FixedTrajectory
from diagnostic_msgs.msg import KeyValue

import tf
from tf.transformations import euler_matrix, translation_matrix, concatenate_matrices

# ignore future deprecated warnings
import warnings
warnings.filterwarnings("ignore", category=FutureWarning)


class VisualServo:
    def __init__(self):
        
        # Camera parameters and control constants
        self.fx = 410.9  # Focal length in x
        self.fy = 410.9  # Focal length in y
        self.cx = 640.0  # Optical center x
        self.cy = 540.0  # Optical center y

        # Load the YOLO model
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('visual_servo_landing')  # Replace with your package name
        model_path = os.path.join(package_path, 'model/yolov5_landing_pad/best.pt')
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)

        # ROS Subscribers and Publishers

        rospy.init_node('visual_servo_landing', anonymous=True)
        odom_sub = rospy.Subscriber('odometry', Odometry, self.odom_callback, queue_size=10)
        rgb_image_sub = Subscriber('camera/color/image_raw', Image)
        depth_image_sub = Subscriber('camera/aligned_depth_to_color/image_raw', Image)
        activate_sub = rospy.Subscriber('visual_servo_switch/switchY', UInt8, self.activate_callback, queue_size=2)
        mode_sub = rospy.Subscriber('visual_servo_switch/switchX', UInt8, self.mode_callback, queue_size=2)
        img_tss = TimeSynchronizer([rgb_image_sub, depth_image_sub], 10)
        img_tss.registerCallback(self.image_callback)

        self.velocity_pub = rospy.Publisher("velocity_setpoint", TwistStamped, queue_size=10)
        self.img_debug_pub = rospy.Publisher("image_debug", Image, queue_size=2)
        self.pose_ctrl_mute_pub = rospy.Publisher("pose_controller/mute_control", Bool, queue_size=100)
        self.fixed_traj_pub = rospy.Publisher("/fixed_trajectory", FixedTrajectory, queue_size=100)

        self.odom = Odometry()
        self.got_odom = False
        self.activate_visual_servo = False
        self.visual_servo_mode = 0 # 0: PBVS, 1: IBVS

    def odom_callback(self, odom):
        self.odom = odom
        self.got_odom = True
    def activate_callback(self, data):
        self.activate_visual_servo = data.data

    def mode_callback(self, data):
        self.visual_servo_mode = data.data

    def image_callback(self, rgb_msg, depth_msg):
        try:
            # Convert the ROS image messages to OpenCV images
            rgb = CvBridge().imgmsg_to_cv2(rgb_msg, "bgr8")
            depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(depth_msg.height, depth_msg.width, -1)
        except Exception as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        x, y, z = self.detect_landing_pad(rgb, depth)

        if self.activate_visual_servo and x is not None:
            if self.visual_servo_mode == 0:
                self.pbvs_control(x, y, z)
            elif self.visual_servo_mode == 1:
                self.ibvs_control(x, y, z)

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

    def detect_landing_pad(self, image, depth):
        ## Use YOLO model to detect the landing pad
        center_x, center_y, detected_z = None, None, None
        if self.model is not None:
            result = self.model(image)
            debug_image = image.copy()
            detections = result.xyxy[0]
            for x1, y1, x2, y2, conf, cls in detections:
                # Format coordinates
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                cv2.rectangle(debug_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                self.publish_debug_image(debug_image)

            if center_x is not None:
                detected_z = depth[int(center_x), int(center_y)].item()
        
        return center_x, center_y, detected_z
    
    def publish_velocity_ibvs(self, vx, vy, vz):
        vel_msg = TwistStamped()
        vel_msg.header.frame_id = "/base_link_stabilized"
        vel_msg.header.stamp = rospy.get_rostime()
        vel_msg.twist.linear.x = vx
        vel_msg.twist.linear.y = vy
        vel_msg.twist.linear.z = vz

        msg = Bool()
        msg.data = True
        
        self.pose_ctrl_mute_pub.publish(msg)
        self.velocity_pub.publish(vel_msg)

    def publish_position_pbvs(self, px, py, pz):
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
        traj.attributes.append(att6)

        self.fixed_traj_pub.publish(traj)

    ######## ASSIGNMENT FUNCTIONS STARTS HERE ########################
    def camera_to_body(self, camera_x, camera_y, camera_z):
        
        # TODO: Fill in this function to implement camera frame to robot body frame 
        #       transformation (rotation followed by translation -- use camera extrinsics)
        
        P_r = np.zeros(3)

        return P_r[0], P_r[1], P_r[2]

    def image_to_camera(self, image_x, image_y, depth):
        
        # TODO: Fill in this function to implement image plane to camera frame 
        #       transformation (use camera intrisics and depth)
        X_c, Y_c, Z_c = 0, 0, 0

        return X_c, Y_c, Z_c
    
    

    def body_to_inertial(self, body_x, body_y, body_z):

        # Convert body frame coordinates to a numpy array
        P_r = np.array([body_x, body_y, body_z, 1])  # 4x1 homogeneous coordinates
        
        # Robot position in the inertial frame
        current_position = np.array([
            self.odom.pose.pose.position.x,
            self.odom.pose.pose.position.y,
            self.odom.pose.pose.position.z
        ])

        # Robot orientation as a quaternion
        current_orientation_quat = self.odom.pose.pose.orientation

        # Roboot orientation as euler angles
        current_orientation_euler = tf.transformations.euler_from_quaternion([
                                                                                current_orientation_quat.x,
                                                                                current_orientation_quat.y,
                                                                                current_orientation_quat.z,
                                                                                current_orientation_quat.w
                                                                            ])              
        # TODO: Fill in this function to implement robot body frame to inertial (world)
        #       frame transformation (use current_position, current_orientation)

        # 1. Create a 4x4 rotation matrix from current_orientation
        
        # 2. Create a 4x4 translation matrix from the current position
        
        # 3. Combine rotation and translation into a single 4x4 transformation matrix

        # 4. Transform the point from the body frame to the inertial frame using the 
        #    4x4 transformation matrix
        P_i = np.zeros(3)

        # Return the x, y, z coordinates in the inertial frame
        return P_i[0], P_i[1], P_i[2]
    
    def pbvs_control(self, x, y, z):
        # Transform from the image plane to the camera frame
        x_c, y_c, z_c = self.image_to_camera(x, y, z)
        # Transform from the camera frame to the body frame
        x_b, y_b, z_b = self.camera_to_body(x_c, y_c, z_c)
        # Transform from the body frame to the inertial frame
        x_i, y_i, z_i = self.body_to_inertial(x_b, y_b, z_b)
        # Publish the position command for the PBVS control system
        self.publish_position_pbvs(x_i, y_i, z_i)

    def ibvs_control(self, x, y, z):
        x_gain = 0.0008
        y_gain = 0.0008
        z_gain = 0.8

        # TODO: Compute error vector in image plane (2x1 e matrix)
        
        # TODO: Compute interaction matrix L (Only positional part of the interaction 
        #       matrix for this assignment [3*2 matrix])

        # TODO: Compute the pseudo-inverse of the interaction matrix ane v_c using this
        #       pseudo-inverse and lambda_gain (needs to be tuned).
        
        v_c = np.zeros(3)
        # Publish the velocity command in camera frame for the IBVS control system
        self.publish_velocity_ibvs(v_c[0], v_c[1], v_c[2])
        


if __name__ == '__main__':
    follower = VisualServo()
    rospy.spin()
