#ifndef _POSE_CONTROLLER_NODE_H_
#define _POSE_CONTROLLER_NODE_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <base/BaseNode.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <pose_controller/pose_controller.h>

class PoseControlNode : public BaseNode 
{
private:

  // Subscribers
  ros::Subscriber _tracking_point_sub, _odometry_sub, _arm_sub, _mute_sub;

  // Publishers
  ros::Publisher _command_pub;

  // Services
  ros::ServiceServer _publish_control_server;

  // TF Listeners
  tf::TransformListener* _tf_listener;

  // Variables
  PoseController* _pose_controller;
  GeoFence _pos_fence;
  double _xy_vel_limit;   
  double _hover_thrust;   
  double _thrust_max;     
  double _thrust_min;     
  double _max_tilt_deg;   
  double _execute_target;
  std::string _target_frame_str;
    
  // Callback functions
  void _tracking_point_callback(const nav_msgs::Odometry &odom);
  void _arm_callback(const std_msgs::Bool &armed);
  void _odometry_callback(const nav_msgs::Odometry &odom);
  void _mute_callback(const std_msgs::Bool &mute);

public:
  PoseControlNode(std::string node_name);
  virtual bool initialize();
  virtual bool execute();
  virtual ~PoseControlNode();
};

#endif
