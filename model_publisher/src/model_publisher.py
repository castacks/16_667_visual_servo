#!/usr/bin/python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
import time

if __name__ == '__main__':
    rospy.init_node('model_publisher')

    model_filename = rospy.get_param('~model_filename', '')
    frame_id = rospy.get_param('~frame_id', 'map')
    if frame_id.startswith('/'):
        frame_id = frame_id[1:]

    r = rospy.get_param('~red', 1.)
    g = rospy.get_param('~green', 1.)
    b = rospy.get_param('~blue', 1.)
    a = rospy.get_param('~alpha', 1.)
    
    scale_x = rospy.get_param('~scale_x', 1.)
    scale_y = rospy.get_param('~scale_y', 1.)
    scale_z = rospy.get_param('~scale_z', 1.)

    mesh_use_embedded_materials = rospy.get_param('~mesh_use_embedded_materials', False)
    
    model_pub = rospy.Publisher('model_viz', Marker, queue_size=10)
    
    rate = rospy.Rate(100.)
    
    marker = Marker()
    marker.action = Marker.ADD
    marker.type = Marker.MESH_RESOURCE
    marker.ns = "model"
    marker.id = 0
    marker.header.frame_id = frame_id
    marker.mesh_resource = "file://" + model_filename
    marker.mesh_use_embedded_materials = mesh_use_embedded_materials
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = a
    
    marker.scale.x = scale_x
    marker.scale.y = scale_y
    marker.scale.z = scale_z

    print(marker.mesh_resource)
    
    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        model_pub.publish(marker)
        
        rate.sleep()
        #time.sleep(0.01)
