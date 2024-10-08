#!/usr/bin/python
from turtle import width
from attr import attr
from matplotlib.pyplot import sca
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, UInt8
from geometry_msgs.msg import Point
from geometry_msgs.msg import WrenchStamped, PoseStamped
import rospy
import tf.transformations as trans
import argparse
#from at_fcs_mavros.msg import Trajectory
#from at_fcs_mavros.msg import Waypoint
#from ca_nav_msgs.msg import XYZVPsi
#from ca_nav_msgs.msg import PathXYZVPsi
from core_trajectory_msgs.msg import WaypointXYZVYaw
from core_trajectory_msgs.msg import TrajectoryXYZVYaw
from core_trajectory_controller.msg import Trajectory
from core_trajectory_msgs.msg import FixedTrajectory
import time
import copy

global odom_x
global odom_y
global odom_z

def get_velocities(traj, velocity, max_acc):
    v_prev = 0.

    for i in range(len(traj.waypoints)):
        j = (i+1) % len(traj.waypoints)
        dx = traj.waypoints[j].position.x - traj.waypoints[i].position.x
        dy = traj.waypoints[j].position.y - traj.waypoints[i].position.y
        dz = traj.waypoints[j].position.z - traj.waypoints[i].position.z

        dist = np.sqrt(dx**2 + dy**2 + dz**2)
        v_limit = np.sqrt(v_prev**2 + 2*max_acc*dist)
        print("vlimit: ",v_limit)
        traj.waypoints[i].velocity = min(velocity, v_limit)
        v_prev = traj.waypoints[i].velocity



def get_racetrack_waypoints(attributes):#length, width, height):
    frame_id = str(attributes['frame_id'])
    length = float(attributes['length'])
    width = float(attributes['width'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    straightaway_length = length - width

    # first straightaway
    xs1 = np.linspace(0, straightaway_length, 20)
    ys1 = np.zeros(xs1.shape)
    yaws1 = np.zeros(xs1.shape)

    # first turn
    t = np.linspace(-np.pi/2, np.pi/2, 30)[1:-1]
    xs2 = width/2.*np.cos(t) + straightaway_length
    ys2 = width/2.*np.sin(t) + width/2.
    xs2d = -width/2.*np.sin(t) # derivative of xs
    ys2d = width/2.*np.cos(t) # derivative of ys
    yaws2 = np.arctan2(ys2d, xs2d)

    # second straightaway
    xs3 = np.linspace(straightaway_length, 0, 20)
    ys3 = width*np.ones(xs3.shape)
    yaws3 = np.pi*np.ones(xs3.shape)

    # second turn
    t = np.linspace(np.pi/2, 3*np.pi/2, 30)[1:-1]
    xs4 = width/2.*np.cos(t)
    ys4 = width/2.*np.sin(t) + width/2.
    yaws4 = yaws2 + np.pi

    xs = np.hstack((xs1, xs2, xs3, xs4))
    ys = np.hstack((ys1, ys2, ys3, ys4))
    yaws = np.hstack((yaws1, yaws2, yaws3, yaws4))

    now = rospy.Time.now()
    for i in range(xs.shape[0]):
        wp = WaypointXYZVYaw()
        wp.position.x = xs[i]
        wp.position.y = ys[i]
        wp.position.z = height
        wp.yaw = yaws[i]

        traj.waypoints.append(wp)

    get_velocities(traj, velocity, max_acceleration)

    return traj


def get_figure8_waypoints(attributes):#length, width, height):
    frame_id = str(attributes['frame_id'])
    length = float(attributes['length'])
    width = float(attributes['width'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    now = rospy.Time.now()

    # figure 8 points
    t = np.linspace(0, 2*np.pi, 100)
    x = np.cos(t) * length - length
    y = np.cos(t)*np.sin(t) * 2*width

    # derivative of figure 8 curve, used to find yaw
    xd = -np.sin(t) * length
    yd = (np.cos(t)**2 - np.sin(t)**2) * 2*width

    for i in range(t.shape[0] - 1):
        x1 = x[i]
        y1 = y[i]
        x2 = x1 + xd[i]
        y2 = y1 + yd[i]

        yaw = np.arctan2(y2 - y1, x2 - x1)

        wp = WaypointXYZVYaw()
        wp.position.x = x1
        wp.position.y = y1
        wp.position.z = height
        wp.yaw = yaw

        traj.waypoints.append(wp)

    get_velocities(traj, velocity, max_acceleration)

    return traj

def get_line_waypoints(attributes):#length, height):
    frame_id = str(attributes['frame_id'])
    y_att = float(attributes['y'])
    height = float(attributes['height'])
    x_att = float(attributes['x'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id

    if x_att == 0:
        for y in np.arange(0, -y_att, -0.1):
            wp = WaypointXYZVYaw()
            wp.position.x = 0
            wp.position.y = y
            wp.position.z = height
            wp.yaw = 0

            traj.waypoints.append(wp)
    else:
        for x in np.arange(0, x_att, 0.1):
            wp = WaypointXYZVYaw()
            wp.position.x = x
            wp.position.y = 0
            wp.position.z = height
            wp.yaw = 0

            traj.waypoints.append(wp)


    get_velocities(traj, velocity, max_acceleration)

    return traj

def get_line_for_wall_writing_waypoints(traj, frame_id, length, height, velocity, max_acceleration, x, y, slope):#length, height):

    for l in np.linspace(0, length, num=10):
        print(l)
        wp = WaypointXYZVYaw()
        wp.position.x = x
        wp.position.y = y + l*np.cos(slope)
        wp.position.z = height + l*np.sin(slope)
        wp.yaw = 0

        traj.waypoints.append(wp)

    get_velocities(traj, velocity, max_acceleration)
    return traj
def get_line_for_get_to_points_waypoints(traj, frame_id, length, height, velocity, max_acceleration, x, y, slope):#length, height):

    for l in np.linspace(0, length, num=10):
        print(l)
        wp = WaypointXYZVYaw()
        wp.position.x = x + l*np.cos(slope)
        wp.position.y = y + l*np.sin(slope)
        wp.position.z = height
        wp.yaw = 0

        traj.waypoints.append(wp)

    get_velocities(traj, velocity, max_acceleration)
    return traj

def get_line_by_two_points_waypoints(traj, frame_id, velocity, max_acceleration, x1, y1, z1, x2, y2, z2):
    p1 = np.array([x1, y1, z1])
    p2 = np.array([x2, y2, z2])
    e = (p2-p1)
    length = np.linalg.norm(e)
    e = e/length
        
    for l in np.linspace(0, length, num=int(length/0.15)):
        print(l)
        wp = WaypointXYZVYaw()
        wp.position.x = p1[0] + l*e[0]
        wp.position.y = p1[1] + l*e[1]
        wp.position.z = p1[2] + l*e[2]
        wp.yaw = 0

        traj.waypoints.append(wp)
        
    wp = WaypointXYZVYaw()
    wp.position.x = p2[0]
    wp.position.y = p2[1]
    wp.position.z = p2[2]
    wp.yaw = 0

    traj.waypoints.append(wp)
    
    get_velocities(traj, velocity, max_acceleration)
    
    print(traj)
    
    return traj


def get_line_from_here_to_point_waypoints(traj, frame_id, velocity, max_acceleration, x, y, z):
    return get_line_by_two_points_waypoints(traj, frame_id, velocity, max_acceleration, odom_x, odom_y, odom_z, x, y, z)

def get_line_two_point_waypoints(attributes):
    frame_id = str(attributes['frame_id'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])
    activate = int(attributes['activate_wrench'])
    force = float(attributes['force'])
    x = float(attributes['x'])
    y = float(attributes['y'])
    z = float(attributes['z'])


    setpoint = WrenchStamped()
    setpoint.header.frame_id = str('ft_sensor')
    setpoint.wrench.force.z = force
    setpoint.wrench.force.y = 0
    setpoint.wrench.force.x = 0

    wrench_force_pub.publish(setpoint)

    activate_topic = UInt8()
    activate_topic.data = activate
    wrench_switch_pub.publish(activate_topic)


    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    traj.header.stamp = rospy.Time.now()
    print('vel: ' + str(velocity) + 'max acc: ' + str(max_acceleration))

    traj = get_line_from_here_to_point_waypoints(traj, frame_id, velocity, max_acceleration, x, y, z)

    # print(traj)
    return traj


def get_line_for_return_home_waypoints(traj, height, velocity, max_acceleration):#length, height):
    global odom_x
    global odom_y
    print("odom y/odom x = ", odom_y/odom_x)
    slope = np.arctan2(odom_y, odom_x)
    length = np.sqrt(odom_x**2 + odom_y**2)
    for l in np.linspace(0, length, num=10):
        wp = WaypointXYZVYaw()
        wp.position.x = odom_x - l*np.cos(slope)
        wp.position.y = odom_y - l*np.sin(slope)
        wp.position.z = height
        wp.yaw = 0

        traj.waypoints.append(wp)

    get_velocities(traj, velocity, max_acceleration)
    return traj

def get_point_waypoints(attributes):#length, height):
    frame_id = str(attributes['frame_id'])
    x = float(attributes['x'])
    y = float(attributes['y'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id

    # add first point
    wp = WaypointXYZVYaw()
    wp.position.x = x
    wp.position.y = y
    wp.position.z = height
    wp.yaw = 0

    traj.waypoints.append(wp)

    get_velocities(traj, velocity, max_acceleration)

    return traj

def get_box_waypoints(attributes):#length, height):
    frame_id = str(attributes['frame_id'])
    length = float(attributes['length'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id

    wp1 = WaypointXYZVYaw()
    wp1.position.x = 0.
    wp1.position.y = 0.
    wp1.position.z = height
    wp1.yaw = 0
    traj.waypoints.append(wp1)

    wp2 = WaypointXYZVYaw()
    wp2.position.x = length
    wp2.position.y = 0.
    wp2.position.z = height
    wp2.yaw = 0
    traj.waypoints.append(wp2)

    wp3 = WaypointXYZVYaw()
    wp3.position.x = length
    wp3.position.y = 0.
    wp3.position.z = height + height
    wp3.yaw = 0
    traj.waypoints.append(wp3)

    wp4 = WaypointXYZVYaw()
    wp4.position.x = 0.
    wp4.position.y = 0.
    wp4.position.z = height + height
    wp4.yaw = 0
    traj.waypoints.append(wp4)

    return traj

def get_vertical_lawnmower_waypoints(attributes):#length, width, height, velocity):
    frame_id = str(attributes['frame_id'])
    length = float(attributes['length'])
    width = float(attributes['width'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id

    for i in range(abs(int(height/width))):
        wp1 = WaypointXYZVYaw()
        wp1.position.x = 0
        wp1.position.y = 0
        wp1.position.z = np.sign(height)*(i+1)*width
        wp1.yaw = 0
        wp1.velocity = 0.1

        wp1_ = WaypointXYZVYaw()
        wp1_.position.x = 0
        wp1_.position.y = 0.5
        wp1_.position.z = np.sign(height)*(i+1)*width
        wp1_.yaw = 0
        wp1_.velocity = velocity

        wp2 = WaypointXYZVYaw()
        wp2.position.x = 0
        wp2.position.y = length
        wp2.position.z = np.sign(height)*(i+1)*width
        wp2.yaw = 0
        wp2.velocity = 0.1

        wp2_ = WaypointXYZVYaw()
        wp2_.position.x = 0
        wp2_.position.y = length - 0.5
        wp2_.position.z = np.sign(height)*(i+1)*width
        wp2_.yaw = 0
        wp2_.velocity = velocity

        if i%2 == 0:
            traj.waypoints.append(wp1)
            wp1_slow = copy.deepcopy(wp1_)
            wp1_slow.velocity = 0.1
            traj.waypoints.append(wp1_slow)
            traj.waypoints.append(wp1_)
            traj.waypoints.append(wp2_)
            traj.waypoints.append(wp2)
        else:
            traj.waypoints.append(wp2)
            wp2_slow = copy.deepcopy(wp2_)
            wp2_slow.velocity = 0.1
            traj.waypoints.append(wp2_slow)
            traj.waypoints.append(wp2_)
            traj.waypoints.append(wp1_)
            traj.waypoints.append(wp1)

    wp = WaypointXYZVYaw()
    wp.position.x = 0
    wp.position.y = 0
    wp.position.z = 0
    wp.yaw = 0
    wp.velocity = 0.5
    traj.waypoints.append(wp)

    return traj

def get_circle_waypoints(attributes):#radius, velocity, frame_id):
    frame_id = str(attributes['frame_id'])
    radius = float(attributes['radius'])
    velocity = float(attributes['velocity'])
    height = 0
    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    traj.header.stamp = rospy.Time.now()

    wp0 = WaypointXYZVYaw()
    wp0.position.x = 0
    wp0.position.y = 0
    wp0.position.z = height
    wp0.yaw = 0
    wp0.velocity = velocity
    traj.waypoints.append(wp0)

    wp1 = WaypointXYZVYaw()
    wp1.position.x = radius
    wp1.position.y = 0
    wp1.position.z = height
    wp1.yaw = 0
    wp1.velocity = velocity
    traj.waypoints.append(wp1)

    for angle in np.arange(0, 2*np.pi, 10.*np.pi/180.):
        wp = WaypointXYZVYaw()
        wp.position.x = radius*np.cos(angle)
        wp.position.y = radius*np.sin(angle)
        wp.position.z = height
        wp.yaw = 0
        wp.velocity = velocity
        traj.waypoints.append(wp)

    wp_end0 = WaypointXYZVYaw()
    wp_end0.position.x = radius
    wp_end0.position.y = 0
    wp_end0.position.z = height
    wp_end0.yaw = 0
    wp_end0.velocity = velocity
    traj.waypoints.append(wp_end0)

    wp_end1 = WaypointXYZVYaw()
    wp_end1.position.x = 0
    wp_end1.position.y = 0
    wp_end1.position.z = height
    wp_end1.yaw = 0
    wp_end1.velocity = velocity
    traj.waypoints.append(wp_end1)

    return traj


def get_wall_painting_waypoints(attributes):
    frame_id = str(attributes['frame_id'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])
    activate = int(attributes['activate_wrench'])
    force = float(attributes['force'])
    width = float(attributes['width'])
    height = float(attributes['height'])

    setpoint = WrenchStamped()
    setpoint.header.frame_id = str('ft_sensor')
    setpoint.wrench.force.z = force
    setpoint.wrench.force.y = 0
    setpoint.wrench.force.x = 0

    wrench_force_pub.publish(setpoint)

    activate_topic = UInt8()
    activate_topic.data = activate
    wrench_switch_pub.publish(activate_topic)


    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    traj.header.stamp = rospy.Time.now()
    print('vel: ' + str(velocity) + 'max acc: ' + str(max_acceleration))

    p1 = np.array([odom_x, odom_y, odom_z])
    p2 = p1 + np.array([0, width, 0])
    p3 = p2 + np.array([0,0,height])
    p4 = p1 + np.array([0,0,height])
    
    traj = get_line_by_two_points_waypoints(traj, frame_id, velocity, max_acceleration, p1[0], p1[1], p1[2], p2[0], p2[1], p2[2])
    traj = get_line_by_two_points_waypoints(traj, frame_id, velocity, max_acceleration, p2[0], p2[1], p2[2], p3[0], p3[1], p3[2])
    traj = get_line_by_two_points_waypoints(traj, frame_id, velocity, max_acceleration, p3[0], p3[1], p3[2], p4[0], p4[1], p4[2])
    # print(traj)
    return traj

def get_boat_landing_waypoints(attributes):
    frame_id = str(attributes['frame_id'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])
    activate = int(attributes['activate_wrench'])
    force = float(attributes['force'])
    width = float(attributes['width'])
    height = float(attributes['height'])

    setpoint = WrenchStamped()
    setpoint.header.frame_id = str('ft_sensor')
    setpoint.wrench.force.z = force
    setpoint.wrench.force.y = 0
    setpoint.wrench.force.x = 0

    wrench_force_pub.publish(setpoint)

    activate_topic = UInt8()
    activate_topic.data = activate
    wrench_switch_pub.publish(activate_topic)


    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    traj.header.stamp = rospy.Time.now()
    print('vel: ' + str(velocity) + 'max acc: ' + str(max_acceleration))

    p1 = np.array([odom_x, odom_y, odom_z])
    initial_h =  odom_z
    p2 = p1 + np.array([0, 0, height])
    p3 = p2 + np.array([width, 0, 0])
    # p4 = p1 + np.array([width, 0, height-initial_h])
    
    traj = get_line_by_two_points_waypoints(traj, frame_id, velocity, max_acceleration, p1[0], p1[1], p1[2], p2[0], p2[1], p2[2])
    traj = get_line_by_two_points_waypoints(traj, frame_id, velocity, max_acceleration, p2[0], p2[1], p2[2], p3[0], p3[1], p3[2])
    # traj = get_line_by_two_points_waypoints(traj, frame_id, velocity, max_acceleration, p3[0], p3[1], p3[2], p4[0], p4[1], p4[2])
    print(traj)
    return traj

def get_wall_writing_waypoints(attributes):#radius, velocity, frame_id):
    frame_id = str(attributes['frame_id'])
    velocity = float(attributes['velocity'])
    height = float(attributes['height'])
    max_acceleration = float(attributes['max_acceleration'])
    scale = float(attributes['scale'])
    activate = int(attributes['activate_wrench'])
    force = float(attributes['force'])
    setpoint = WrenchStamped()
    setpoint.header.frame_id = str('ft_sensor')
    setpoint.wrench.force.z = force
    setpoint.wrench.force.y = 0
    setpoint.wrench.force.x = 0

    wrench_force_pub.publish(setpoint)

    activate_topic = UInt8()
    activate_topic.data = activate
    wrench_switch_pub.publish(activate_topic)

    x = 9.3

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    traj.header.stamp = rospy.Time.now()
    print('vel: ' + str(velocity) + 'max acc: ' + str(max_acceleration))


    traj = get_line_for_wall_writing_waypoints(traj, frame_id, scale, height, velocity, max_acceleration, x, 0, 2*3.1415/3)

    p = traj.waypoints[-1].position
    traj = get_line_for_wall_writing_waypoints(traj, frame_id, scale, p.z, velocity, max_acceleration, x, p.y, -2*3.1415/3)

    p = traj.waypoints[-1].position
    traj = get_line_for_wall_writing_waypoints(traj, frame_id, 0.5*scale, p.z, velocity, max_acceleration, x, p.y, 3.1415/3)

    p = traj.waypoints[-1].position
    traj = get_line_for_wall_writing_waypoints(traj, frame_id, 0.5*scale, p.z, velocity, max_acceleration, x, p.y, 0)

    #traj.waypoints.append(traj2.waypoints)

    # print(traj)
    return traj

def get_to_point_waypoints(attributes):#radius, velocity, frame_id):
    frame_id = str(attributes['frame_id'])
    velocity = float(attributes['velocity'])
    height = float(attributes['height'])
    max_acceleration = float(attributes['max_acceleration'])
    x = float(attributes['x'])
    y = float(attributes['y'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    traj.header.stamp = rospy.Time.now()


    traj = get_line_for_get_to_points_waypoints(traj, frame_id, 0.5, height, velocity, max_acceleration, x, y, 3.1415/3)

    # print(traj)
    return traj

def get_return_home_waypoints(attributes):#radius, velocity, frame_id):
    frame_id = str('world')
    velocity = 0.35
    height = float(attributes['height'])
    max_acceleration = float(attributes['max_acceleration'])
    x = float(0)
    y = float(0)

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    traj.header.stamp = rospy.Time.now()
    activate = UInt8()
    activate.data = 0
    wrench_switch_pub.publish(activate)

    traj = get_line_for_return_home_waypoints(traj, height, velocity, max_acceleration)

    activate_topic = UInt8()
    activate_topic.data = 0
    X_switch_pub.publish(activate_topic)
    Y_switch_pub.publish(activate_topic)
    Z_switch_pub.publish(activate_topic)


    # print(traj)
    return traj

def get_wrench_activation(attributes):
    activate = int(attributes['activate_wrench'])
    print("activate: s",activate)
    force = float(attributes['force'])
    setpoint = WrenchStamped()
    setpoint.header.frame_id = str('ft_sensor')
    setpoint.wrench.force.z = force
    setpoint.wrench.force.y = 0
    setpoint.wrench.force.x = 0

    wrench_force_pub.publish(setpoint)

    activate_topic = UInt8()
    activate_topic.data = activate
    wrench_switch_pub.publish(activate_topic)

    

def get_visual_point_activation(attributes):
    activate = int(attributes['activate'])
    height = float(attributes['height'])

    visual_point_msg = PoseStamped()
    visual_point_msg.header.frame_id = "base_link"
    visual_point_msg.header.stamp = rospy.get_rostime()
    visual_point_msg.pose.position.y = height


    tracking_visual_point_pub.publish(visual_point_msg)
    
    activate_topic = UInt8()
    activate_topic.data = activate
    Z_switch_pub.publish(activate_topic)
    
    print("activate visual servo: s",activate)

def get_visual_line_activation(attributes):
    activate = int(attributes['activate'])
    Line_switch = UInt8()
    Line_switch.data = activate
    
    activate_topic = UInt8()
    activate_topic.data = activate
    line_switch_pub.publish(activate_topic)



def fixed_trajectory_callback(msg):
    attributes = {}

    for key_value in msg.attributes:
        attributes[key_value.key] = key_value.value

    trajectory_msg = None
    
    if msg.type == 'Circle':
        trajectory_msg = get_circle_waypoints(attributes)
    elif msg.type == 'Line':
        trajectory_msg = get_line_waypoints(attributes)
    elif msg.type == 'Point':
        trajectory_msg = get_point_waypoints(attributes)
    elif msg.type == 'WallWriting':
        trajectory_msg = get_wall_writing_waypoints(attributes)
    elif msg.type == 'ReturnHome':
        trajectory_msg = get_return_home_waypoints(attributes)
    elif msg.type == 'ActivateWrench':
        trajectory_msg = get_wrench_activation(attributes)
    elif msg.type == 'VisualPoint':
        trajectory_msg = get_visual_point_activation(attributes)
    elif msg.type == 'VisualLine':
        trajectory_msg = get_visual_line_activation(attributes)
    elif msg.type == 'OpenloopThrust':
        pass
    elif msg.type == 'WallPainting':
        trajectory_msg = get_wall_painting_waypoints(attributes)
    elif msg.type == 'BoatLanding':
        trajectory_msg = get_boat_landing_waypoints(attributes)

    if trajectory_msg != None:
        trajectory_track_pub.publish(trajectory_msg)
        print('trajectory pub')
    else:
        print('No trajectory sent.')

def odometry_callback(msg):
    global odom_x
    global odom_y
    global odom_z
    odom_x = msg.pose.pose.position.x
    odom_y = msg.pose.pose.position.y
    odom_z = msg.pose.pose.position.z

if __name__ == '__main__':
    rospy.init_node('fixed_trajectory_generator')

    fixed_trajectory_sub = rospy.Subscriber('fixed_trajectory', FixedTrajectory, fixed_trajectory_callback)
    odom_sub = rospy.Subscriber('/odometry', Odometry, odometry_callback)

    trajectory_track_pub = rospy.Publisher('/trajectory_track', TrajectoryXYZVYaw, queue_size=1)
    wrench_switch_pub = rospy.Publisher('/wrench_controller/switchX', UInt8, queue_size=1)
    line_switch_pub = rospy.Publisher('/Lines/Switch', UInt8, queue_size=1)
    wrench_force_pub = rospy.Publisher('/ft_setpoint', WrenchStamped, queue_size=1)
    tracking_visual_point_pub = rospy.Publisher('/tracking_visual_point', PoseStamped, queue_size=1)
    Z_switch_pub = rospy.Publisher('/wrench_controller/switchZ', UInt8, queue_size=1)
    Y_switch_pub = rospy.Publisher('/wrench_controller/switchY', UInt8, queue_size=1)
    X_switch_pub = wrench_switch_pub
    rospy.spin()
