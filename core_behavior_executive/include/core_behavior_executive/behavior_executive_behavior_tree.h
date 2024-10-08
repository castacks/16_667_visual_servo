#ifndef _CORE_BEHAVIOR_EXECUTIVE_H_
#define _CORE_BEHAVIOR_EXECUTIVE_H_

#include <base/BaseNode.h>
#include <string>
#include <behavior_tree/behavior_tree.h>
#include <behavior_tree_msgs/Status.h>
#include <behavior_tree_msgs/BehaviorTreeCommand.h>
#include <behavior_tree_msgs/BehaviorTreeCommands.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <core_trajectory_msgs/FixedTrajectory.h>
#include <core_takeoff_landing_planner/TakeoffLandingCommand.h>
#include <core_trajectory_controller/TrajectoryMode.h>
#include <core_drone_interface/DroneCommand.h>
#include <std_srvs/Empty.h>

class BehaviorExecutiveBehaviorTree : public BaseNode {
private:

  // variables
  std::string takeoff_state, landing_state;
  core_trajectory_msgs::FixedTrajectory fixed_trajectory;

  // conditions
  std::vector<bt::Condition*> conditions;
  bt::Condition* takeoff_commanded_condition;
  bt::Condition* land_commanded_condition;
  bt::Condition* pause_commanded_condition;
  bt::Condition* fixed_trajectory_commanded_condition;
  bt::Condition* explore_commanded_condition;
  bt::Condition* autonomously_explore_commanded_condition;
  bt::Condition* offboard_commanded_condition;
  bt::Condition* arm_commanded_condition;
  bt::Condition* disarm_commanded_condition;
  bt::Condition* rewind_commanded_condition;
  bt::Condition* offboard_mode_condition;
  bt::Condition* armed_condition;
  bt::Condition* takeoff_complete_condition;
  bt::Condition* landed_condition;
  bt::Condition* in_air_condition;
  
  // actions
  std::vector<bt::Action*> actions;
  bt::Action* takeoff_action;
  bt::Action* land_action;
  bt::Action* pause_action;
  bt::Action* follow_fixed_trajectory_action;
  bt::Action* explore_action;
  bt::Action* request_control_action;
  bt::Action* arm_action;
  bt::Action* disarm_action;
  bt::Action* rewind_action;

  // services
  ros::ServiceClient takeoff_landing_client;
  ros::ServiceClient trajectory_mode_client;
  ros::ServiceClient drone_command_client;
  ros::ServiceClient x_reset_integrator_client, y_reset_integrator_client, z_reset_integrator_client, yaw_reset_integrator_client;
  ros::ServiceClient vx_reset_integrator_client, vy_reset_integrator_client, vz_reset_integrator_client, yawrate_reset_integrator_client;
  
  // publishers
  ros::Publisher fixed_trajectory_pub;
  ros::Publisher in_air_pub;
  
  // subscribers
  ros::Subscriber behavior_tree_command_sub;
  ros::Subscriber takeoff_state_sub;
  ros::Subscriber landing_state_sub;
  ros::Subscriber fixed_trajectory_command_sub;
  ros::Subscriber has_control_sub;
  ros::Subscriber is_armed_sub;
  
  // callbacks
  void behavior_tree_command_callback(behavior_tree_msgs::BehaviorTreeCommands msg);
  void takeoff_state_callback(std_msgs::String msg);
  void landing_state_callback(std_msgs::String msg);
  void fixed_trajectory_command_callback(core_trajectory_msgs::FixedTrajectory msg);
  void has_control_callback(std_msgs::Bool msg);
  void is_armed_callback(std_msgs::Bool msg);
  
public:
  BehaviorExecutiveBehaviorTree(std::string node_name);
  
  virtual bool initialize();
  virtual bool execute();
  virtual ~BehaviorExecutiveBehaviorTree();

};


#endif
