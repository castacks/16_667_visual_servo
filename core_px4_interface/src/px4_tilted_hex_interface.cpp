#include <pluginlib/class_list_macros.h>
#include <core_px4_interface/px4_tilted_hex_interface.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

PX4TiltedHexInterface::PX4TiltedHexInterface()
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // init parameters
  float command_pose_pub_rate = pnh.param("command_pose_pub_rate", 20.0);
  default_mpc_xy_cruise = pnh.param("command_pose_pub_rate", 5.0);
  default_mpc_vel_manual = pnh.param("command_pose_pub_rate", 10.0);

  // init subscribers
  state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &PX4TiltedHexInterface::state_callback, this);
  pixhawk_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &PX4TiltedHexInterface::pixhawk_pose_callback, this);
  odom_sub = nh.subscribe<nav_msgs::Odometry>("odometry", 10, &PX4TiltedHexInterface::odom_callback, this);
  att_mode_sub = nh.subscribe<core_px4_interface::AttMode>("controller/commands/set_att_mode", 10, &PX4TiltedHexInterface::att_mode_callback, this);
  max_xy_vel_sub = nh.subscribe<std_msgs::Float32>("controller/commands/set_max_xy_speed", 10, &PX4TiltedHexInterface::max_xy_speed_callback, this);

  command_publisher_timer = nh.createTimer(ros::Duration(1./command_pose_pub_rate), &PX4TiltedHexInterface::command_publisher_timer_callback, this);

  // init publishers
  attitude_target_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
  pose_sp_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  velocity_sp_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 10);

  // init services
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  set_param_client = nh.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");

  // init variables
  offboard_srv.request.custom_mode = "OFFBOARD";
  arm_srv.request.value = true;
  disarm_srv.request.value = false;
  pixhawk_yaw = 0;
  got_odom = false;
}

bool PX4TiltedHexInterface::request_att_mode(int mode)
{
  ROS_WARN("Switching att to %d", mode);
  mavros_msgs::ParamSet set_param_srv;
  set_param_srv.request.param_id = "OMNI_ATT_MODE";
  set_param_srv.request.value.integer = mode;
  return set_param_client.call(set_param_srv) && set_param_srv.response.success;
}

bool PX4TiltedHexInterface::request_max_xy_speed(float speed) 
{
  mavros_msgs::ParamSet set_param_srv;
  ROS_WARN("Changing speed to %0.2f", speed);
  set_param_srv.request.param_id = "MPC_XY_VEL_MAX";
  set_param_srv.request.value.real = speed;
  if (!set_param_client.call(set_param_srv) && set_param_srv.response.success)
  {
    ROS_ERROR("MPC_XY_VEL_MAX parameter not set.");
    return false;
  }

  set_param_srv.request.param_id = "MPC_VEL_MANUAL";
  set_param_srv.request.value.real = std::min(speed, default_mpc_vel_manual);
  if (!set_param_client.call(set_param_srv) && set_param_srv.response.success)
  {
    ROS_ERROR("MPC_VEL_MANUAL parameter not set.");
    return false;
  }

  set_param_srv.request.param_id = "MPC_XY_CRUISE";
  set_param_srv.request.value.real = std::min(speed, default_mpc_xy_cruise);
  if (!set_param_client.call(set_param_srv) && set_param_srv.response.success)
  {
    ROS_ERROR("MPC_XY_CRUISE parameter not set.");
    return false;
  }
  return true;
}

bool PX4TiltedHexInterface::request_control() 
{
    bool success = true;
  if(current_state.mode != "OFFBOARD")
    success = set_mode_client.call(offboard_srv) && offboard_srv.response.mode_sent;
  return success;
}

bool PX4TiltedHexInterface::arm()
{
  bool success = true;
  if(!current_state.armed)
    success = arming_client.call(arm_srv) && arm_srv.response.success;
  return success;
}

bool PX4TiltedHexInterface::disarm()
{
  bool success = true;
  if(current_state.armed)
    success = arming_client.call(disarm_srv) && disarm_srv.response.success;
  return success;
}

bool PX4TiltedHexInterface::is_armed()
{
  return current_state.armed;
}

bool PX4TiltedHexInterface::has_control()
{
  return current_state.mode == "OFFBOARD";
}

/**********************************************************************
 * Callback Functions                                                 *
 * *******************************************************************/

void PX4TiltedHexInterface::state_callback(mavros_msgs::State msg)
{
  current_state = msg;
  if (has_control()) 
  {
    is_offboard = true;
  }
  else if (is_offboard)
  {
    is_offboard = false;
    command_type = CommandType::None;
  }
}

void PX4TiltedHexInterface::pixhawk_pose_callback(geometry_msgs::PoseStamped pose)
{
  if (!has_control())
    current_pose = pose;
  tf::Quaternion q(pose.pose.orientation.x,
		   pose.pose.orientation.y,
		   pose.pose.orientation.z,
		   pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  pixhawk_yaw = yaw;
}


void PX4TiltedHexInterface::odom_callback(nav_msgs::Odometry msg)
{
  got_odom = true;
  odom = msg;
}

void PX4TiltedHexInterface::att_mode_callback(core_px4_interface::AttMode msg)
{
  request_att_mode(msg.Mode);
}

void PX4TiltedHexInterface::max_xy_speed_callback(std_msgs::Float32 msg) 
{
  request_max_xy_speed(msg.data);
}

/**********************************************************************
 * Command Functions                                                  *
 * *******************************************************************/

void PX4TiltedHexInterface::command_pose(geometry_msgs::PoseStamped msg)
{
  last_command_pose = msg;
  if (is_offboard)
    command_type = CommandType::Position;
  publish_pose(msg);
}

void PX4TiltedHexInterface::command_velocity(geometry_msgs::TwistStamped msg)
{
  last_command_velocity = msg;
  if (is_offboard)
    command_type = CommandType::Velocity;
  publish_velocity(msg);
}

void PX4TiltedHexInterface::command_roll_pitch_yawrate_thrust(mav_msgs::RollPitchYawrateThrust msg)
{
  mavros_msgs::AttitudeTarget att_msg;
  att_msg.header.frame_id = msg.header.frame_id;
  att_msg.header.stamp = ros::Time::now();
  att_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | 
                  mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE;
  tf::Quaternion q;
  q.setRPY(msg.roll, msg.pitch, pixhawk_yaw);
  att_msg.body_rate.z = msg.yaw_rate;
  att_msg.thrust = msg.thrust.z;
  
  att_msg.orientation.x = q.x();
  att_msg.orientation.y = q.y();
  att_msg.orientation.z = q.z();
  att_msg.orientation.w = q.w();

  last_command_attitude_target = att_msg;
  if (is_offboard)
    command_type = CommandType::AttitudeThrust;
  publish_attitude_target(att_msg);
}

void PX4TiltedHexInterface::command_rate_thrust(mav_msgs::RateThrust msg)
{
  mavros_msgs::AttitudeTarget att_msg;
  att_msg.header.frame_id = msg.header.frame_id;
  att_msg.header.stamp = ros::Time::now();
  att_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

  att_msg.body_rate.x = msg.angular_rates.x;
  att_msg.body_rate.y = msg.angular_rates.y;
  att_msg.body_rate.z = msg.angular_rates.z;
  att_msg.thrust = msg.thrust.z;

  last_command_attitude_target = att_msg;
  if (is_offboard)
    command_type = CommandType::AttitudeThrust;
  publish_attitude_target(att_msg);
}

void PX4TiltedHexInterface::command_attitude_thrust(mav_msgs::AttitudeThrust msg)
{
  mavros_msgs::AttitudeTarget att_msg;
  att_msg.header.frame_id = msg.header.frame_id;
  att_msg.header.stamp = ros::Time::now();
  att_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | 
                      mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                      mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

  att_msg.thrust = msg.thrust.z;
  att_msg.orientation.x = msg.attitude.x;
  att_msg.orientation.y = msg.attitude.y;
  att_msg.orientation.z = msg.attitude.z;
  att_msg.orientation.w = msg.attitude.w;

  last_command_attitude_target = att_msg;
  if (is_offboard)
    command_type = CommandType::AttitudeThrust;
  publish_attitude_target(att_msg);
}

/**********************************************************************
 * Publish Functions                                                  *
 * *******************************************************************/

void PX4TiltedHexInterface::publish_pose(geometry_msgs::PoseStamped msg)
{
  pose_sp_pub.publish(msg);
}

void PX4TiltedHexInterface::publish_velocity(geometry_msgs::TwistStamped msg)
{
  velocity_sp_pub.publish(msg);
}

void PX4TiltedHexInterface::publish_attitude_target(mavros_msgs::AttitudeTarget msg)
{
  attitude_target_pub.publish(msg);
}

/**********************************************************************
 * Timer                                                              *
 * *******************************************************************/

void PX4TiltedHexInterface::command_publisher_timer_callback(const ros::TimerEvent &te)
{
  switch (command_type)
  {
  case CommandType::Position:
    last_command_pose.header.stamp = ros::Time::now();
    publish_pose(last_command_pose);
    break;

  case CommandType::Velocity:
    last_command_velocity.header.stamp = ros::Time::now();
    publish_velocity(last_command_velocity);
    break;

  case CommandType::RateThrust:
  case CommandType::RPYrThrust:
  case CommandType::AttitudeThrust:
    last_command_attitude_target.header.stamp = ros::Time::now();
    publish_attitude_target(last_command_attitude_target);
    break;

  default:
    current_pose.header.stamp = ros::Time::now();
    publish_pose(current_pose);
    break;
  }
}

PLUGINLIB_EXPORT_CLASS(PX4TiltedHexInterface, DroneInterface)
