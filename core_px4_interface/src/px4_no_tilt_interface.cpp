#include <pluginlib/class_list_macros.h>
#include <core_px4_interface/px4_no_tilt_interface.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

PX4NoTiltInterface::PX4NoTiltInterface(){
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // init parameters
  float mavros_pose_pub_rate = pnh.param("mavros_pose_pub_rate", 20.);
  
  // init subscribers
  state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &PX4NoTiltInterface::state_callback, this);
  pixhawk_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &PX4NoTiltInterface::pixhawk_pose_callback, this);
  odom_sub = nh.subscribe<nav_msgs::Odometry>("odometry", 10, &PX4NoTiltInterface::odom_callback, this);

  mavros_pose_timer = nh.createTimer(ros::Duration(1./mavros_pose_pub_rate), &PX4NoTiltInterface::mavros_pose_timer_callback, this);

  // init publishers
  rate_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
  mavros_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);
  
  // init services
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // init variables
  offboard_srv.request.custom_mode = "OFFBOARD";
  arm_srv.request.value = true;
  disarm_srv.request.value = false;
  pixhawk_yaw = 0;
  got_odom = false;
}

bool PX4NoTiltInterface::request_control(){
  bool success = true;
  if(current_state.mode != "OFFBOARD")
    success = set_mode_client.call(offboard_srv) && offboard_srv.response.mode_sent;
  return success;
}

bool PX4NoTiltInterface::arm(){
  bool success = true;
  if(!current_state.armed)
    success = arming_client.call(arm_srv) && arm_srv.response.success;
  return success;
}

bool PX4NoTiltInterface::disarm(){
  bool success = true;
  if(current_state.armed)
    success = arming_client.call(disarm_srv) && disarm_srv.response.success;
  return success;
}

bool PX4NoTiltInterface::is_armed(){
  return current_state.armed;
}

bool PX4NoTiltInterface::has_control(){
  return current_state.mode == "OFFBOARD";
}

void PX4NoTiltInterface::command_roll_pitch_yawrate_thrust(mav_msgs::RollPitchYawrateThrust msg){
  mavros_msgs::AttitudeTarget att;
  //att.header.frame_id = "world";
  att.header.stamp = ros::Time::now();
  att.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE;
  tf::Quaternion q;
  q.setRPY(msg.roll, msg.pitch, pixhawk_yaw);
  att.body_rate.z = msg.yaw_rate;
  att.thrust = msg.thrust.z;
  
  att.orientation.x = q.x();
  att.orientation.y = q.y();
  att.orientation.z = q.z();
  att.orientation.w = q.w();

  rate_thrust_pub.publish(att);
}

void PX4NoTiltInterface::command_rate_thrust(mav_msgs::RateThrust msg){
  mavros_msgs::AttitudeTarget att;
  //att.header.frame_id = "world";
  att.header.stamp = ros::Time::now();
  att.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

  att.body_rate.x = msg.angular_rates.x;
  att.body_rate.y = msg.angular_rates.y;
  att.body_rate.z = msg.angular_rates.z;
  att.thrust = msg.thrust.z;

  rate_thrust_pub.publish(att);
}

void PX4NoTiltInterface::state_callback(mavros_msgs::State msg){
  current_state = msg;
}

void PX4NoTiltInterface::pixhawk_pose_callback(geometry_msgs::PoseStamped pose){
  tf::Quaternion q(pose.pose.orientation.x,
		   pose.pose.orientation.y,
		   pose.pose.orientation.z,
		   pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  pixhawk_yaw = yaw;
}


void PX4NoTiltInterface::odom_callback(nav_msgs::Odometry msg){
  got_odom = true;
  odom = msg;
}

void PX4NoTiltInterface::mavros_pose_timer_callback(const ros::TimerEvent& te){
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();//odom.header.stamp;
  pose.header.frame_id = "map";
  pose.pose.position = odom.pose.pose.position;
  pose.pose.orientation = odom.pose.pose.orientation;

  mavros_pose_pub.publish(pose);
}

PLUGINLIB_EXPORT_CLASS(PX4NoTiltInterface, DroneInterface)
