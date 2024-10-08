#ifndef _GAZEBO_INTERFACE_NODE_H_
#define _GAZEBO_INTERFACE_NODE_H_

#include <base/BaseNode.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <gazebo_interface/gazebo_interface.h>

class GazeboInterfaceNode : public BaseNode {
private:

public:
  GazeboInterfaceNode(std::string node_name);
  
  virtual bool initialize();
  virtual bool execute();
  virtual ~GazeboInterfaceNode();

};


#endif
