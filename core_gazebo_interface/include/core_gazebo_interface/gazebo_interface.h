#ifndef _GAZEBO_INTERFACE_H_
#define _GAZEBO_INTERFACE_H_

#include <core_drone_interface/drone_interface.h>

class GazeboInterface : public DroneInterface {
private:

  // publishers
  ros::Publisher cmd_vel_pub;
  
public:
  GazeboInterface();
  
  virtual bool request_control();
  virtual bool arm();
  virtual bool disarm();
  virtual bool is_armed();
  virtual bool has_control();

  virtual void command_velocity(geometry_msgs::TwistStamped msg);
};



#endif
