#ifndef _DRONE_INTERFACE_NODE_H_
#define _DRONE_INTERFACE_NODE_H_

#include <base/BaseNode.h>
#include <string>
#include <core_drone_interface/drone_interface.h>
#include <core_drone_interface/DroneCommand.h>
#include <std_msgs/Bool.h>

class DroneInterfaceNode : public BaseNode {
private:
  // parameters
  std::string drone_interface_name;
  
  // variables
  boost::shared_ptr<DroneInterface> drone_interface;

  // services
  ros::ServiceServer drone_command_server;
  
  // subscribers
  ros::Subscriber attitude_thrust_sub, rate_thrust_sub, roll_pitch_yawrate_thrust_sub,
    torque_thrust_sub, velocity_sub, pose_sub;

  // publishers
  ros::Publisher is_armed_pub, has_control_pub;

  // callbacks
  void attitude_thrust_callback(mav_msgs::AttitudeThrust msg);
  void rate_thrust_callback(mav_msgs::RateThrust msg);
  void roll_pitch_yawrate_thrust_callback(mav_msgs::RollPitchYawrateThrust msg);
  void torque_thrust_callback(mav_msgs::TorqueThrust msg);
  void velocity_callback(geometry_msgs::TwistStamped msg);
  void pose_callback(geometry_msgs::PoseStamped msg);

  bool drone_command(core_drone_interface::DroneCommand::Request& req,
		     core_drone_interface::DroneCommand::Response& res);
  
public:
  DroneInterfaceNode(std::string node_name);
  
  virtual bool initialize();
  virtual bool execute();
  virtual ~DroneInterfaceNode();

};


#endif
