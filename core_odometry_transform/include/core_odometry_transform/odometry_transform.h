#ifndef _ODOMETRY_TRANSFORM_H_
#define _ODOMETRY_TRANSFORM_H_

#include <base/BaseNode.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class OdometryTransform : public BaseNode {
private:
  enum OdometryOutputType {NONE, TRANSFORMED, RESTAMPED};
  
  // parameters
  bool convert_odometry_to_transform,
    convert_odometry_to_stabilized_transform,
    restamp_now;
  OdometryOutputType odometry_output_type;
  std::string transform_name, new_frame_id, new_child_frame_id;
  bool rotate_orientation;
  double rotate_orientation_x;
  double rotate_orientation_y;
  double rotate_orientation_z;
  double rotate_orientation_w;
  tf::Transform rotate_orientation_tf;

  // subscribers
  ros::Subscriber odometry_sub;
  tf::TransformListener* listener;
  
  // publishers
  ros::Publisher odometry_pub;
  tf::TransformBroadcaster* broadcaster;

  // callbacks
  void odometry_callback(nav_msgs::Odometry odom);
  
public:
  OdometryTransform(std::string node_name);
  
  virtual bool initialize();
  virtual bool execute();
  virtual ~OdometryTransform();
  
};


#endif
