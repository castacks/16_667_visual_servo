#ifndef _PX4_TILTED_HEX_INTERFACE_H_
#define _PX4_TILTED_HEX_INTERFACE_H_

#include <core_drone_interface/drone_interface.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ParamSet.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <core_px4_interface/AttMode.h>
#include <std_msgs/Float32.h>

class PX4TiltedHexInterface : public DroneInterface 
{
private:

  enum CommandType
  {
    None,
    Position,
    Velocity,
    AttitudeThrust,
    RPYrThrust,
    RateThrust
  };

  // parameters
  float default_mpc_xy_cruise, default_mpc_vel_manual;

  // variables
  mavros_msgs::State current_state;
  mavros_msgs::SetMode offboard_srv;
  mavros_msgs::CommandBool arm_srv, disarm_srv;
  double pixhawk_yaw;
  bool got_odom;
  nav_msgs::Odometry odom;
  geometry_msgs::PoseStamped current_pose;
  bool is_offboard = false;

  CommandType command_type = CommandType::None;
  geometry_msgs::PoseStamped last_command_pose;
  geometry_msgs::TwistStamped last_command_velocity;
  mavros_msgs::AttitudeTarget last_command_attitude_target;

  // publishers
  ros::Publisher attitude_target_pub, external_pose_pub, pose_sp_pub, velocity_sp_pub;

  // services
  ros::ServiceClient arming_client, set_mode_client, set_param_client;
  
  // subcribers
  ros::Subscriber state_sub, pixhawk_pose_sub, odom_sub, att_mode_sub, max_xy_vel_sub;
  ros::Timer command_publisher_timer;

  // callbacks
  void att_mode_callback(core_px4_interface::AttMode);
  void max_xy_speed_callback(std_msgs::Float32);
  void state_callback(mavros_msgs::State msg);
  void pixhawk_pose_callback(geometry_msgs::PoseStamped pose);
  void odom_callback(nav_msgs::Odometry msg);
  void command_publisher_timer_callback(const ros::TimerEvent& te);
  
  // Other functions
  void publish_pose(geometry_msgs::PoseStamped msg);
  void publish_velocity(geometry_msgs::TwistStamped msg);
  void publish_attitude_target(mavros_msgs::AttitudeTarget msg);

  bool request_att_mode(int mode);
  bool request_max_xy_speed(float speed);

public:

  PX4TiltedHexInterface();
  
  virtual bool request_control();
  virtual bool arm();
  virtual bool disarm();
  virtual bool is_armed();
  virtual bool has_control();

  virtual void command_roll_pitch_yawrate_thrust(mav_msgs::RollPitchYawrateThrust msg);
  virtual void command_attitude_thrust(mav_msgs::AttitudeThrust msg);
  virtual void command_rate_thrust(mav_msgs::RateThrust msg);
  virtual void command_pose(geometry_msgs::PoseStamped);
  virtual void command_velocity(geometry_msgs::TwistStamped);
};

#endif
