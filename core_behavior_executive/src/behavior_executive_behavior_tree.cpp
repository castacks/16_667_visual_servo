#include <base/BaseNode.h>
#include <string>
#include "core_behavior_executive/behavior_executive_behavior_tree.h"

BehaviorExecutiveBehaviorTree::BehaviorExecutiveBehaviorTree(std::string node_name)
  : BaseNode(node_name){
}

bool BehaviorExecutiveBehaviorTree::initialize(){
  ros::NodeHandle* nh = get_node_handle();
  ros::NodeHandle* pnh = get_private_node_handle();

  // init conditions
  takeoff_commanded_condition = new bt::Condition("Takeoff Commanded");
  land_commanded_condition = new bt::Condition("Land Commanded");
  pause_commanded_condition = new bt::Condition("Pause Commanded");
  fixed_trajectory_commanded_condition = new bt::Condition("Fixed Trajectory Commanded");
  explore_commanded_condition = new bt::Condition("Autonomously Explore Commanded");
  autonomously_explore_commanded_condition = new bt::Condition("Explore Commanded");
  offboard_commanded_condition = new bt::Condition("Offboard Commanded");
  arm_commanded_condition = new bt::Condition("Arm Commanded");
  disarm_commanded_condition = new bt::Condition("Disarm Commanded");
  rewind_commanded_condition = new bt::Condition("Rewind Commanded");
  offboard_mode_condition = new bt::Condition("Offboard Mode");
  armed_condition = new bt::Condition("Armed");
  takeoff_complete_condition = new bt::Condition("Takeoff Complete");
  landed_condition = new bt::Condition("Landed");
  in_air_condition = new bt::Condition("In Air");

  conditions.push_back(takeoff_commanded_condition);
  conditions.push_back(land_commanded_condition);
  conditions.push_back(pause_commanded_condition);
  conditions.push_back(fixed_trajectory_commanded_condition);
  conditions.push_back(explore_commanded_condition);
  conditions.push_back(autonomously_explore_commanded_condition);
  conditions.push_back(offboard_commanded_condition);
  conditions.push_back(arm_commanded_condition);
  conditions.push_back(disarm_commanded_condition);
  conditions.push_back(rewind_commanded_condition);
  conditions.push_back(offboard_mode_condition);
  conditions.push_back(armed_condition);
  conditions.push_back(takeoff_complete_condition);
  conditions.push_back(landed_condition);
  conditions.push_back(in_air_condition);
  
  // init actions
  takeoff_action = new bt::Action("Takeoff");
  land_action = new bt::Action("Land");
  pause_action = new bt::Action("Pause");
  follow_fixed_trajectory_action = new bt::Action("Follow Fixed Trajectory");
  explore_action = new bt::Action("Explore");
  request_control_action = new bt::Action("Request Control");
  arm_action = new bt::Action("Arm");
  disarm_action = new bt::Action("Disarm");
  rewind_action = new bt::Action("Rewind");

  actions.push_back(takeoff_action);
  actions.push_back(land_action);
  actions.push_back(pause_action);
  actions.push_back(follow_fixed_trajectory_action);
  actions.push_back(explore_action);
  actions.push_back(request_control_action);
  actions.push_back(arm_action);
  actions.push_back(disarm_action);
  actions.push_back(rewind_action);

  // init services
  takeoff_landing_client = nh->serviceClient<core_takeoff_landing_planner::TakeoffLandingCommand>("set_takeoff_landing_command");
  trajectory_mode_client = nh->serviceClient<core_trajectory_controller::TrajectoryMode>("set_trajectory_mode");
  drone_command_client = nh->serviceClient<core_drone_interface::DroneCommand>("drone_command");
  x_reset_integrator_client = nh->serviceClient<std_srvs::Empty>("pose_controller/x/reset_integrator");
  y_reset_integrator_client = nh->serviceClient<std_srvs::Empty>("pose_controller/y/reset_integrator");
  z_reset_integrator_client = nh->serviceClient<std_srvs::Empty>("pose_controller/z/reset_integrator");
  yaw_reset_integrator_client = nh->serviceClient<std_srvs::Empty>("pose_controller/yaw/reset_integrator");
  vx_reset_integrator_client = nh->serviceClient<std_srvs::Empty>("velocity_controller/vx/reset_integrator");
  vy_reset_integrator_client = nh->serviceClient<std_srvs::Empty>("velocity_controller/vy/reset_integrator");
  vz_reset_integrator_client = nh->serviceClient<std_srvs::Empty>("velocity_controller/vz/reset_integrator");
  yawrate_reset_integrator_client = nh->serviceClient<std_srvs::Empty>("velocity_controller/yawrate/reset_integrator");

  // init publishers
  fixed_trajectory_pub = nh->advertise<core_trajectory_msgs::FixedTrajectory>("fixed_trajectory", 10);
  in_air_pub = nh->advertise<std_msgs::Bool>("in_air", 1);
  
  // init subscribers
  behavior_tree_command_sub = nh->subscribe("behavior_tree_commands", 10, &BehaviorExecutiveBehaviorTree::behavior_tree_command_callback, this);
  takeoff_state_sub = nh->subscribe("takeoff_state", 10, &BehaviorExecutiveBehaviorTree::takeoff_state_callback, this);
  landing_state_sub = nh->subscribe("landing_state", 10, &BehaviorExecutiveBehaviorTree::landing_state_callback, this);
  fixed_trajectory_command_sub = nh->subscribe("fixed_trajectory_command", 10, &BehaviorExecutiveBehaviorTree::fixed_trajectory_command_callback, this);
  has_control_sub = nh->subscribe("has_control", 10, &BehaviorExecutiveBehaviorTree::has_control_callback, this);
  is_armed_sub = nh->subscribe("is_armed", 10, &BehaviorExecutiveBehaviorTree::is_armed_callback, this);
  
  
  return true;
}

bool BehaviorExecutiveBehaviorTree::execute(){
  // request control action
  if(request_control_action->is_active()){
    if(request_control_action->active_has_changed()){
      core_drone_interface::DroneCommand drone_command_srv;
      drone_command_srv.request.command = core_drone_interface::DroneCommand::Request::REQUEST_CONTROL;
      drone_command_client.call(drone_command_srv);

      if(drone_command_srv.response.success){
	ROS_INFO("SUCCESS REQUEST CONTROL");
	request_control_action->set_success();
      }
      else{
	ROS_INFO("FAILED REQUEST CONTROL");
	request_control_action->set_failure();
      }
    }
  }

  // arm action
  if(arm_action->is_active()){
    if(arm_action->active_has_changed()){
      // reset integrators before arming
      std_srvs::Empty reset_srv;
      x_reset_integrator_client.call(reset_srv);
      y_reset_integrator_client.call(reset_srv);
      z_reset_integrator_client.call(reset_srv);
      yaw_reset_integrator_client.call(reset_srv);
      vx_reset_integrator_client.call(reset_srv);
      vy_reset_integrator_client.call(reset_srv);
      vz_reset_integrator_client.call(reset_srv);
      yawrate_reset_integrator_client.call(reset_srv);
      
      // set takeoff and land conditions to false before arming
      takeoff_complete_condition->set(false);
      landed_condition->set(false);

      // arm
      core_drone_interface::DroneCommand drone_command_srv;
      drone_command_srv.request.command = core_drone_interface::DroneCommand::Request::ARM;
      drone_command_client.call(drone_command_srv);

      if(drone_command_srv.response.success){
	ROS_INFO("ARMED SUCCESS");
	arm_action->set_success();
      }
      else{
	ROS_INFO("ARMED FAILURE");
	arm_action->set_failure();
      }

      core_trajectory_controller::TrajectoryMode srv;
      srv.request.mode = core_trajectory_controller::TrajectoryMode::Request::PAUSE;
      trajectory_mode_client.call(srv);
    }
  }

  // disarm action
  if(disarm_action->is_active()){
    if(disarm_action->active_has_changed()){
      in_air_condition->set(false);
      core_drone_interface::DroneCommand drone_command_srv;
      drone_command_srv.request.command = core_drone_interface::DroneCommand::Request::DISARM;
      drone_command_client.call(drone_command_srv);

      if(drone_command_srv.response.success){
	ROS_INFO("DISARMED SUCCESS");
	disarm_action->set_success();

	
	// reset integrators after disarming
	std_srvs::Empty reset_srv;
	x_reset_integrator_client.call(reset_srv);
	y_reset_integrator_client.call(reset_srv);
	z_reset_integrator_client.call(reset_srv);
	yaw_reset_integrator_client.call(reset_srv);
	vx_reset_integrator_client.call(reset_srv);
	vy_reset_integrator_client.call(reset_srv);
	vz_reset_integrator_client.call(reset_srv);
	yawrate_reset_integrator_client.call(reset_srv);
	
	// set the tracking point to be at the robot's position
	core_trajectory_controller::TrajectoryMode srv;
	srv.request.mode = core_trajectory_controller::TrajectoryMode::Request::ROBOT_POSE;
	trajectory_mode_client.call(srv);
      }
      else{
	ROS_INFO("DISARMED FAILURE");
	disarm_action->set_failure();
      }
    }
  }
  
  // takeoff action
  if(takeoff_action->is_active()){
    takeoff_action->set_running();
    in_air_condition->set(true);
    
    if(takeoff_action->active_has_changed()){
      core_takeoff_landing_planner::TakeoffLandingCommand takeoff_srv;
      takeoff_srv.request.command = core_takeoff_landing_planner::TakeoffLandingCommand::Request::TAKEOFF;
      takeoff_landing_client.call(takeoff_srv);
    }

    if(takeoff_state == "COMPLETE"){
      takeoff_complete_condition->set(true);
      takeoff_action->set_success();
    }
  }

  // land action
  if(land_action->is_active()){
    land_action->set_running();

    if(land_action->active_has_changed()){
      core_takeoff_landing_planner::TakeoffLandingCommand land_srv;
      land_srv.request.command = core_takeoff_landing_planner::TakeoffLandingCommand::Request::LAND;
      takeoff_landing_client.call(land_srv);
    }

    if(landing_state == "COMPLETE"){
      landed_condition->set(true);
      land_action->set_success();
    }
  }

  // pause action
  if(pause_action->is_active()){
    pause_action->set_running();

    if(pause_action->active_has_changed()){
      core_trajectory_controller::TrajectoryMode srv;
      srv.request.mode = core_trajectory_controller::TrajectoryMode::Request::PAUSE;
      trajectory_mode_client.call(srv);
    }
  }

  // rewind action
  if(rewind_action->is_active()){
    rewind_action->set_running();

    if(rewind_action->active_has_changed()){
      core_trajectory_controller::TrajectoryMode srv;
      srv.request.mode = core_trajectory_controller::TrajectoryMode::Request::REWIND;
      trajectory_mode_client.call(srv);
    }
  }

  // follow fixed trajectory action
  if(follow_fixed_trajectory_action->is_active()){
    follow_fixed_trajectory_action->set_running();

    if(follow_fixed_trajectory_action->active_has_changed()){
      core_trajectory_controller::TrajectoryMode srv;
      srv.request.mode = core_trajectory_controller::TrajectoryMode::Request::TRACK;
      trajectory_mode_client.call(srv);

      fixed_trajectory_pub.publish(fixed_trajectory);
    }
  }
  
  // explore action
  if(explore_action->is_active()){
    explore_action->set_running();

    if(explore_action->active_has_changed()){
      core_trajectory_controller::TrajectoryMode srv;
      srv.request.mode = core_trajectory_controller::TrajectoryMode::Request::SEGMENT;
      trajectory_mode_client.call(srv);
    }
  }
  
  for(int i = 0; i < conditions.size(); i++)
    conditions[i]->publish();
  for(int i = 0; i < actions.size(); i++)
    if(actions[i]->is_active())
      actions[i]->publish();

  std_msgs::Bool in_air_msg;
  in_air_msg.data = in_air_condition->get();
  in_air_pub.publish(in_air_msg);
  
  return true;
}

void BehaviorExecutiveBehaviorTree::behavior_tree_command_callback(behavior_tree_msgs::BehaviorTreeCommands msg){
  for(int i = 0; i < msg.commands.size(); i++){
    std::string condition_name = msg.commands[i].condition_name;
    int status = msg.commands[i].status;
    
    for(int j = 0; j < conditions.size(); j++){
      bt::Condition* condition = conditions[j];
      if(condition_name == condition->get_label()){
	if(status == behavior_tree_msgs::Status::SUCCESS)
	  condition->set(true);
	else if(status == behavior_tree_msgs::Status::FAILURE)
	  condition->set(false);
      }
    }
  }
}

void BehaviorExecutiveBehaviorTree::takeoff_state_callback(std_msgs::String msg){
  takeoff_state = msg.data;
}

void BehaviorExecutiveBehaviorTree::landing_state_callback(std_msgs::String msg){
  landing_state = msg.data;
}

void BehaviorExecutiveBehaviorTree::fixed_trajectory_command_callback(core_trajectory_msgs::FixedTrajectory msg){
  fixed_trajectory = msg;
}

void BehaviorExecutiveBehaviorTree::has_control_callback(std_msgs::Bool msg){
  offboard_mode_condition->set(msg.data);
}

void BehaviorExecutiveBehaviorTree::is_armed_callback(std_msgs::Bool msg){
  armed_condition->set(msg.data);
}

BehaviorExecutiveBehaviorTree::~BehaviorExecutiveBehaviorTree(){
}

BaseNode* BaseNode::get(){
  BehaviorExecutiveBehaviorTree* core_behavior_executive = new BehaviorExecutiveBehaviorTree("BehaviorExecutiveBehaviorTree");
  return core_behavior_executive;
}
