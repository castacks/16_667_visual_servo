#include <pluginlib/class_list_macros.h>
#include <core_px4_interface/px4_interface.h>
#include <ros/ros.h>

PX4Interface::PX4Interface(){
  ros::NodeHandle nh;
  
  // init subscribers
  state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &PX4Interface::state_callback, this);

  // init publishers
  rate_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
  
  // init services
  arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // init variables
  offboard_srv.request.custom_mode = "OFFBOARD";
  arm_srv.request.value = true;
  disarm_srv.request.value = false;
}

bool PX4Interface::request_control(){
  ROS_INFO("TRYING TO ENTER OFFBOARD MODE");
  bool success = false;
  if(current_state.mode != "OFFBOARD"){
    success = set_mode_client.call(offboard_srv) && offboard_srv.response.mode_sent;
  }
  ROS_INFO_STREAM("OFFBOARD " << success);
  return success;
}

bool PX4Interface::arm(){
  bool success = true;
  ROS_INFO_STREAM("PREARM CHECK: " << (int)!current_state.armed);
  if(!current_state.armed){
    success = arming_client.call(arm_srv) && arm_srv.response.success;
    ROS_INFO("TRYING TO ARM");
  }
  ROS_INFO("ARMED");
  return success;
}

bool PX4Interface::disarm(){
  bool success = true;
  ROS_INFO_STREAM("PREDISARM CHECK: " << (int)current_state.armed);
  if(current_state.armed){
    success = arming_client.call(disarm_srv) && disarm_srv.response.success;
    ROS_INFO("TRYING TO DISARM");
  }
  ROS_INFO("DISARMED");
  return success;
}

bool PX4Interface::is_armed(){
  return current_state.armed;
}

bool PX4Interface::has_control(){
  return current_state.mode == "OFFBOARD";
}

void PX4Interface::command_roll_pitch_yawrate_thrust(mav_msgs::RollPitchYawrateThrust msg){
  mavros_msgs::AttitudeTarget att;
  att.body_rate.x = msg.pitch;
  att.body_rate.y = msg.roll;
  att.body_rate.z = msg.thrust.z;
  att.thrust = msg.yaw_rate;

  att.header.frame_id = "world";
  att.header.stamp = ros::Time::now();
  att.orientation.w = 1;

  rate_thrust_pub.publish(att);
}

void PX4Interface::state_callback(mavros_msgs::State msg){
  current_state = msg;
}

PLUGINLIB_EXPORT_CLASS(PX4Interface, DroneInterface)
