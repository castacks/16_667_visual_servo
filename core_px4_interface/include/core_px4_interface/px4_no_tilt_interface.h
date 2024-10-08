#ifndef _GAZEBO_INTERFACE_H_
#define _GAZEBO_INTERFACE_H_

#include <core_drone_interface/drone_interface.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class PX4NoTiltInterface : public DroneInterface {
private:

  // variables
  mavros_msgs::State current_state;
  mavros_msgs::SetMode offboard_srv;
  mavros_msgs::CommandBool arm_srv, disarm_srv;
  double pixhawk_yaw;
  bool got_odom;
  nav_msgs::Odometry odom;

  // publishers
  ros::Publisher rate_thrust_pub, mavros_pose_pub;
  
  // services
  ros::ServiceClient arming_client, set_mode_client;
  
  // subcribers
  ros::Subscriber state_sub, pixhawk_pose_sub, odom_sub;
  ros::Timer mavros_pose_timer;

  // callbacks
  void state_callback(mavros_msgs::State msg);
  void pixhawk_pose_callback(geometry_msgs::PoseStamped pose);
  void odom_callback(nav_msgs::Odometry msg);
  void mavros_pose_timer_callback(const ros::TimerEvent& te);
  
public:
  PX4NoTiltInterface();
  
  virtual bool request_control();
  virtual bool arm();
  virtual bool disarm();
  virtual bool is_armed();
  virtual bool has_control();

  virtual void command_roll_pitch_yawrate_thrust(mav_msgs::RollPitchYawrateThrust msg);
  virtual void command_rate_thrust(mav_msgs::RateThrust msg);
};



#endif
