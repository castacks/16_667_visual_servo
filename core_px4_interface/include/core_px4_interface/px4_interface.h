#ifndef _GAZEBO_INTERFACE_H_
#define _GAZEBO_INTERFACE_H_

#include <core_drone_interface/drone_interface.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>

class PX4Interface : public DroneInterface {
private:

  // variables
  mavros_msgs::State current_state;
  mavros_msgs::SetMode offboard_srv;
  mavros_msgs::CommandBool arm_srv, disarm_srv;

  // publishers
  ros::Publisher rate_thrust_pub;
  
  // services
  ros::ServiceClient arming_client, set_mode_client;
  
  // subcribers
  ros::Subscriber state_sub;

  // callbacks
  void state_callback(mavros_msgs::State msg);
  
public:
  PX4Interface();
  
  virtual bool request_control();
  virtual bool arm();
  virtual bool disarm();
  virtual bool is_armed();
  virtual bool has_control();

  virtual void command_roll_pitch_yawrate_thrust(mav_msgs::RollPitchYawrateThrust msg);
};



#endif
