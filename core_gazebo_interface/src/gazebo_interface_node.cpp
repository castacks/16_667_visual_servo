#include <base/BaseNode.h>
#include <string>
#include "gazebo_interface/gazebo_interface_node.h"

GazeboInterfaceNode::GazeboInterfaceNode(std::string node_name)
  : BaseNode(node_name){
}

bool GazeboInterfaceNode::initialize(){
  ros::NodeHandle* nh = get_node_handle();
  ros::NodeHandle* pnh = get_private_node_handle();
  
  return true;
}

bool GazeboInterfaceNode::execute(){
  
  return true;
}

GazeboInterfaceNode::~GazeboInterfaceNode(){
}

BaseNode* BaseNode::get(){
  GazeboInterfaceNode* gazebo_interface_node = new GazeboInterfaceNode("GazeboInterfaceNode");
  return gazebo_interface_node;
}
