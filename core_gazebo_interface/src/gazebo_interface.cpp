#include <pluginlib/class_list_macros.h>
#include <core_gazebo_interface/gazebo_interface.h>
#include <ros/ros.h>

GazeboInterface::GazeboInterface(){
  ros::NodeHandle nh;
  
  // init publishers
  cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("cmd_vel", 10);
}

bool GazeboInterface::request_control(){
  ROS_INFO("REQUEST CONTROL");
  return true;
}

bool GazeboInterface::arm(){
  ROS_INFO("ARM");
  return true;
}

bool GazeboInterface::disarm(){
  ROS_INFO("DISARM");
  return true;
}

bool GazeboInterface::is_armed(){
  return true;
}

bool GazeboInterface::has_control(){
  return true;
}

void GazeboInterface::command_velocity(geometry_msgs::TwistStamped msg){
  cmd_vel_pub.publish(msg);
}

PLUGINLIB_EXPORT_CLASS(GazeboInterface, DroneInterface)
