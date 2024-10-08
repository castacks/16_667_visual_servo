#include <base/BaseNode.h>
#include <string>
#include <core_takeoff_landing_planner/takeoff_landing_planner.h>

TakeoffLandingPlanner::TakeoffLandingPlanner(std::string node_name)
  : BaseNode(node_name){
}

bool TakeoffLandingPlanner::initialize(){
  ros::NodeHandle* nh = get_node_handle();
  ros::NodeHandle* pnh = get_private_node_handle();

  // init parameters
  takeoff_height = pnh->param("takeoff_height", 0.5);
  takeoff_landing_velocity = pnh->param("takeoff_landing_velocity", 0.3);
  takeoff_acceptance_distance = pnh->param("takeoff_acceptance_distance", 0.1);
  takeoff_acceptance_time = pnh->param("takeoff_acceptance_time", 2.0);
  landing_stationary_distance = pnh->param("landing_stationary_distance", 0.02);
  landing_acceptance_time = pnh->param("landing_acceptance_time", 5.0);
  takeoff_path_roll = pnh->param("takeoff_path_roll", 0.)*M_PI/180.;
  takeoff_path_pitch = pnh->param("takeoff_path_pitch", 0.)*M_PI/180.;
  takeoff_path_relative_to_orientation = pnh->param("takeoff_path_relative_to_orientation", false);
  
  
  // init subscribers
  this->listener = new tf::TransformListener();
  completion_percentage_sub = nh->subscribe("trajectory_completion_percentage", 10, &TakeoffLandingPlanner::completion_percentage_callback, this);
  tracking_point_sub = nh->subscribe("tracking_point", 10, &TakeoffLandingPlanner::tracking_point_callback, this);
  robot_odom_sub = nh->subscribe("odometry", 10, &TakeoffLandingPlanner::robot_odom_callback, this);

  // init publishers
  traj_track_pub = nh->advertise<core_trajectory_msgs::TrajectoryXYZVYaw>("trajectory_track", 10);
  takeoff_state_pub = nh->advertise<std_msgs::String>("takeoff_state", 10);
  landing_state_pub = nh->advertise<std_msgs::String>("landing_state", 10);
  
  // init services
  command_server = nh->advertiseService("set_takeoff_landing_command", &TakeoffLandingPlanner::set_takeoff_landing_command, this);
  traj_mode_client = nh->serviceClient<core_trajectory_controller::TrajectoryMode>("set_trajectory_mode");
  
  // init variables
  got_completion_percentage = false;
  got_tracking_point = false;
  got_robot_odom = false;
  track_mode_srv.request.mode = core_trajectory_controller::TrajectoryMode::Request::TRACK;
  takeoff_traj_gen = new TakeoffTrajectory(takeoff_height, takeoff_landing_velocity, takeoff_path_roll, takeoff_path_pitch, takeoff_path_relative_to_orientation);
  landing_traj_gen = new TakeoffTrajectory(-10000., takeoff_landing_velocity);
  current_command = core_takeoff_landing_planner::TakeoffLandingCommand::Request::NONE;

  completion_percentage = 0.f;
  takeoff_is_newly_active = true;
  land_is_newly_active = true;
  takeoff_distance_check = false;
  
  return true;
}

bool TakeoffLandingPlanner::execute(){
  if(!got_completion_percentage || !got_tracking_point || !got_robot_odom)
    return true;

  std_msgs::String takeoff_state, landing_state;
  takeoff_state.data = landing_state.data = "NONE";

  // takeoff
  if(current_command == core_takeoff_landing_planner::TakeoffLandingCommand::Request::TAKEOFF){
    takeoff_state.data = "TAKING_OFF";
        
    if(completion_percentage >= 100.f){
      // get distance between tracking point and robot odom
      float distance = std::numeric_limits<float>::max();
      try{
	tf::StampedTransform transform;
	listener->waitForTransform(tracking_point_odom.header.frame_id, robot_odom.header.frame_id, robot_odom.header.stamp, ros::Duration(0.1));
	listener->lookupTransform(tracking_point_odom.header.frame_id, robot_odom.header.frame_id, robot_odom.header.stamp, transform);
	tf::Vector3 tracking_point_position = tflib::to_tf(tracking_point_odom.pose.pose.position);
	tf::Vector3 robot_odom_position = transform*tflib::to_tf(robot_odom.pose.pose.position);
	
	distance = tracking_point_position.distance(robot_odom_position);
      }
      catch(const tf::TransformException& ex){
	ROS_ERROR_STREAM("TransformException in TakeoffMonitor: " << ex.what());
      }

      //ROS_INFO_STREAM("distance: " << distance);
      
      // check if distance meets the takeoff acceptance threshold
      if(distance <= takeoff_acceptance_distance){
	if(!takeoff_distance_check){
	  takeoff_distance_check = true;
	  takeoff_acceptance_start = robot_odom.header.stamp;
	}
	  
	// check if the distance threshold has been met for the required amount of time
	//ROS_INFO_STREAM("duration: " << (robot_odom.header.stamp - takeoff_acceptance_start).toSec() << " / " << takeoff_acceptance_time);
	if((robot_odom.header.stamp - takeoff_acceptance_start).toSec() >= takeoff_acceptance_time){
	  takeoff_state.data = "COMPLETE";
	}
      }
      else
	takeoff_distance_check = false;
    }
  }
  
  
  // land
  if(current_command == core_takeoff_landing_planner::TakeoffLandingCommand::Request::LAND){
    landing_state.data = "LANDING";

    // check if the robot has been moving
    if(robot_odoms.size() > 2){
      nav_msgs::Odometry initial_odom = robot_odoms.front();
      float time_diff = (robot_odom.header.stamp - initial_odom.header.stamp).toSec();
      
      //landing_detected = time_diff > landing_acceptance_time;
      bool time_check = time_diff > landing_acceptance_time;
      while(!robot_odoms.empty() && (robot_odom.header.stamp - robot_odoms.front().header.stamp).toSec() > landing_acceptance_time){
	initial_odom = robot_odoms.front();
	robot_odoms.pop_front();
      }
      robot_odoms.push_front(initial_odom);

      bool distance_check = false;
      for(auto it = robot_odoms.begin(); time_check && it != robot_odoms.end(); it++){
	float distance = tflib::to_tf(robot_odoms.begin()->pose.pose.position).distance(tflib::to_tf(it->pose.pose.position));
	distance_check = distance <= landing_stationary_distance;
	if(!distance_check){
	  //ROS_INFO_STREAM("distance: " << distance);
	  break;
	}
      }

      //ROS_INFO_STREAM("LANDING CHECK: " << time_diff << " " << time_check << " " << distance_check);
      
      if(time_check && distance_check)
	landing_state.data = "COMPLETE";
    }
  }

  takeoff_state_pub.publish(takeoff_state);
  landing_state_pub.publish(landing_state);
  
  return true;
}


// callbacks
void TakeoffLandingPlanner::completion_percentage_callback(std_msgs::Float32 msg){
  got_completion_percentage = true;
  completion_percentage = msg.data;
}

void TakeoffLandingPlanner::tracking_point_callback(nav_msgs::Odometry msg){
  got_tracking_point = true;
  tracking_point_odom = msg;
}

void TakeoffLandingPlanner::robot_odom_callback(nav_msgs::Odometry msg){
  got_robot_odom = true;
  robot_odom = msg;

  if(current_command == core_takeoff_landing_planner::TakeoffLandingCommand::Request::LAND)
    robot_odoms.push_back(msg);
}

bool TakeoffLandingPlanner::set_takeoff_landing_command(core_takeoff_landing_planner::TakeoffLandingCommand::Request& request,
							core_takeoff_landing_planner::TakeoffLandingCommand::Response& response){
  current_command = request.command;

  if(current_command == core_takeoff_landing_planner::TakeoffLandingCommand::Request::NONE){
    
  }
  else if(current_command == core_takeoff_landing_planner::TakeoffLandingCommand::Request::TAKEOFF){
    // put the trajectory controller into track mode
    traj_mode_client.call(track_mode_srv);
    // publish a takeoff trajectory
    nav_msgs::Odometry takeoff_starting_point = tracking_point_odom;
    if(got_robot_odom && takeoff_path_relative_to_orientation)
      takeoff_starting_point.pose.pose.orientation = robot_odom.pose.pose.orientation;
    
    traj_track_pub.publish(takeoff_traj_gen->get_trajectory(takeoff_starting_point));
  }
  else if(current_command == core_takeoff_landing_planner::TakeoffLandingCommand::Request::LAND){
    robot_odoms.clear();
    // put the trajectory controller into track mode
    traj_mode_client.call(track_mode_srv);
    // publish a landing trajectory
    traj_track_pub.publish(landing_traj_gen->get_trajectory(tracking_point_odom));
  }
  
  response.accepted = true;
  return true;
}

TakeoffLandingPlanner::~TakeoffLandingPlanner(){
}

BaseNode* BaseNode::get(){
  TakeoffLandingPlanner* core_takeoff_landing_planner = new TakeoffLandingPlanner("TakeoffLandingPlanner");
  return core_takeoff_landing_planner;
}
