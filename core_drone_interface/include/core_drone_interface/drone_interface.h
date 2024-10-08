#ifndef _DRONE_INTERFACE_H_
#define _DRONE_INTERFACE_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/RateThrust.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/TorqueThrust.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

class DroneInterface {
private:
  
  
public:
  virtual bool request_control() = 0;
  virtual bool arm() = 0;
  virtual bool disarm() = 0;
  virtual bool is_armed() = 0;
  virtual bool has_control() = 0;
  
  // command functions
  virtual void command_attitude_thrust(mav_msgs::AttitudeThrust){
    ROS_ERROR("command_attitude_thrust WAS CALLED, BUT IS NOT IMPLEMENTED.");
  }
  
  virtual void command_rate_thrust(mav_msgs::RateThrust){
    ROS_ERROR("command_rate_thrust WAS CALLED, BUT IS NOT IMPLEMENTED.");
  }
  
  virtual void command_roll_pitch_yawrate_thrust(mav_msgs::RollPitchYawrateThrust){
    ROS_ERROR("command_roll_pitch_yawrate_thrust WAS CALLED, BUT IS NOT IMPLEMENTED.");
  }
  
  virtual void command_torque_thrust(mav_msgs::TorqueThrust){
    ROS_ERROR("command_torque_thrust WAS CALLED, BUT IS NOT IMPLEMENTED.");
  }
  
  virtual void command_velocity(geometry_msgs::TwistStamped){
    ROS_ERROR("command_velocity WAS CALLED, BUT IS NOT IMPLEMENTED.");
  }
  
  virtual void command_pose(geometry_msgs::PoseStamped){
    ROS_ERROR("command_position WAS CALLED, BUT IS NOT IMPLEMENTED.");
  }
  
protected:
  DroneInterface(){}
};

#endif
