#include <base/BaseNode.h>
#include <string>
#include <core_drone_interface/drone_interface_node.h>

#include <pluginlib/class_loader.h>

DroneInterfaceNode::DroneInterfaceNode(std::string node_name)
  : BaseNode(node_name){
}

bool DroneInterfaceNode::initialize(){
  ros::NodeHandle* nh = get_node_handle();
  ros::NodeHandle* pnh = get_private_node_handle();

  // init parameters
  drone_interface_name = pnh->param("drone_interface", std::string("DroneInterface"));
  
  // load a drone interface
  pluginlib::ClassLoader<DroneInterface> drone_interface_loader("core_drone_interface", "DroneInterface");
  try{
    drone_interface = drone_interface_loader.createInstance(drone_interface_name);
  }
  catch(pluginlib::PluginlibException& ex){
    ROS_ERROR("The DroneInterface plugin failed to load. Error: %s", ex.what());
  }

  // init services
  drone_command_server = nh->advertiseService("drone_command", &DroneInterfaceNode::drone_command, this);
  
  // init subscribers
  attitude_thrust_sub = nh->subscribe("attitude_thrust_command", 10, &DroneInterfaceNode::attitude_thrust_callback, this);
  rate_thrust_sub = nh->subscribe("rate_thrust_command", 10, &DroneInterfaceNode::rate_thrust_callback, this);
  roll_pitch_yawrate_thrust_sub = nh->subscribe("roll_pitch_yawrate_thrust_command", 10, &DroneInterfaceNode::roll_pitch_yawrate_thrust_callback, this);
  torque_thrust_sub = nh->subscribe("torque_thrust_command", 10, &DroneInterfaceNode::torque_thrust_callback, this);
  velocity_sub = nh->subscribe("velocity_command", 10, &DroneInterfaceNode::velocity_callback, this);
  pose_sub = nh->subscribe("pose_command", 10, &DroneInterfaceNode::pose_callback, this);

  // init publishers
  is_armed_pub = nh->advertise<std_msgs::Bool>("is_armed", 1);
  has_control_pub = nh->advertise<std_msgs::Bool>("has_control", 1);
  
  return true;
}

bool DroneInterfaceNode::execute(){
  std_msgs::Bool is_armed;
  is_armed.data = drone_interface->is_armed();
  is_armed_pub.publish(is_armed);
  
  std_msgs::Bool has_control;
  has_control.data = drone_interface->has_control();
  has_control_pub.publish(has_control);
  
  return true;
}

bool DroneInterfaceNode::drone_command(core_drone_interface::DroneCommand::Request& req,
				       core_drone_interface::DroneCommand::Response& res){
  switch(req.command){
  case core_drone_interface::DroneCommand::Request::REQUEST_CONTROL:
    res.success = drone_interface->request_control();
    break;
  case core_drone_interface::DroneCommand::Request::ARM:
    res.success = drone_interface->arm();
    break;
  case core_drone_interface::DroneCommand::Request::DISARM:
    res.success = drone_interface->disarm();
    break;
  }

  return true;
}

void DroneInterfaceNode::attitude_thrust_callback(mav_msgs::AttitudeThrust msg){
  drone_interface->command_attitude_thrust(msg);
}

void DroneInterfaceNode::rate_thrust_callback(mav_msgs::RateThrust msg){
  drone_interface->command_rate_thrust(msg);
}

void DroneInterfaceNode::roll_pitch_yawrate_thrust_callback(mav_msgs::RollPitchYawrateThrust msg){
  drone_interface->command_roll_pitch_yawrate_thrust(msg);
}

void DroneInterfaceNode::torque_thrust_callback(mav_msgs::TorqueThrust msg){
  drone_interface->command_torque_thrust(msg);
}

void DroneInterfaceNode::velocity_callback(geometry_msgs::TwistStamped msg){
  drone_interface->command_velocity(msg);
}

void DroneInterfaceNode::pose_callback(geometry_msgs::PoseStamped msg){
  drone_interface->command_pose(msg);
}

DroneInterfaceNode::~DroneInterfaceNode(){
}

BaseNode* BaseNode::get(){
  DroneInterfaceNode* drone_interface_node = new DroneInterfaceNode("DroneInterfaceNode");
  return drone_interface_node;
}
