#ifndef _TAKEOFF_LANDING_PLANNER_H_
#define _TAKEOFF_LANDING_PLANNER_H_

#include <base/BaseNode.h>
#include <core_trajectory_controller/TrajectoryMode.h>
#include <core_trajectory_library/trajectory_library.h>
#include <tflib/tflib.h>
#include <behavior_tree/behavior_tree.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <string>
#include <core_takeoff_landing_planner/TakeoffLandingCommand.h>

class TakeoffLandingPlanner : public BaseNode {
private:

  // parameters
  float takeoff_height, takeoff_landing_velocity;
  float takeoff_acceptance_distance, takeoff_acceptance_time;
  float landing_stationary_distance, landing_acceptance_time;
  float takeoff_path_roll, takeoff_path_pitch;
  bool takeoff_path_relative_to_orientation;
  
  // variables
  bool got_completion_percentage, got_tracking_point, got_robot_odom;
  nav_msgs::Odometry tracking_point_odom, robot_odom;
  float completion_percentage;
  core_trajectory_controller::TrajectoryMode track_mode_srv;
  uint8_t current_command;

  // takeoff variables
  bool takeoff_is_newly_active;
  bool takeoff_distance_check;
  ros::Time takeoff_acceptance_start;
  TakeoffTrajectory* takeoff_traj_gen;

  // land variables
  bool land_is_newly_active;
  std::list<nav_msgs::Odometry> robot_odoms;
  TakeoffTrajectory* landing_traj_gen;
  
  // subscribers
  ros::Subscriber completion_percentage_sub, tracking_point_sub, robot_odom_sub;
  tf::TransformListener* listener;

  // publishers
  ros::Publisher traj_track_pub, takeoff_state_pub, landing_state_pub;

  // services
  ros::ServiceServer command_server;
  ros::ServiceClient traj_mode_client;

  // callbacks
  void completion_percentage_callback(std_msgs::Float32 msg);
  void tracking_point_callback(nav_msgs::Odometry msg);
  void robot_odom_callback(nav_msgs::Odometry msg);

  bool set_takeoff_landing_command(core_takeoff_landing_planner::TakeoffLandingCommand::Request& request,
				   core_takeoff_landing_planner::TakeoffLandingCommand::Response& response);
  
public:
  TakeoffLandingPlanner(std::string node_name);
  
  virtual bool initialize();
  virtual bool execute();
  virtual ~TakeoffLandingPlanner();

};


#endif
