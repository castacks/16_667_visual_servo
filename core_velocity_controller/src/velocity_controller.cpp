#include <ros/ros.h>
#include <base/BaseNode.h>
#include <core_pid_controller/pid_controller.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_listener.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <std_msgs/Bool.h>

#define DEBUG

class VelocityControlNode : public BaseNode
{
private:
  PIDController *vx_controller;
  PIDController *vy_controller;
  PIDController *vz_controller;
  PIDController *yawrate_controller;

  ros::Subscriber odom_sub, twist_sub, mute_sub;
  tf::TransformListener *listener;

  ros::Publisher command_pub;

  ros::ServiceServer publish_control_server;

  nav_msgs::Odometry odom;
  bool got_odom, got_twist;
  bool should_publish;

  double hover_thrust;

  tf::Vector3 setpoint_vel_target_frame, setpoint_ang_vel_target_frame;
  tf::Vector3 actual_vel_target_frame, actual_ang_vel_target_frame;

  std::string target_frame;

public:
  VelocityControlNode();
  virtual bool initialize();
  virtual bool execute();
  virtual ~VelocityControlNode();

  std::tuple<tf::Quaternion, double> CalculateAttitudeThrust(tf::Vector3 thrust_sp);
  void odom_callback(nav_msgs::Odometry odom);
  void twist_callback(geometry_msgs::TwistStamped twist);
  void mute_callback(const std_msgs::Bool &mute);

  bool publish_control_callback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
};

VelocityControlNode::VelocityControlNode()
    : BaseNode("velocity_control_node")
{
}

bool VelocityControlNode::initialize()
{
  ros::NodeHandle *nh = get_node_handle();
  ros::NodeHandle *pnh = get_private_node_handle();

  // init params
  target_frame = pnh->param(std::string("target_frame"), std::string("base_link_stabilized"));

  got_odom = false;
  got_twist = false;
  should_publish = true;

  // init controllers
  vx_controller = new PIDController("~/vx");
  vy_controller = new PIDController("~/vy");
  vz_controller = new PIDController("~/vz");
  yawrate_controller = new PIDController("~/yawrate");
  yawrate_controller->set_calculate_error_func(calculate_error_angle);

  // init subscribers
  twist_sub = nh->subscribe("velocity_setpoint", 10, &VelocityControlNode::twist_callback, this);
  odom_sub = nh->subscribe("odometry", 10, &VelocityControlNode::odom_callback, this);
  mute_sub = nh->subscribe("pose_controller/mute_control", 10, &VelocityControlNode::mute_callback, this);
  listener = new tf::TransformListener();

  // init publishers
  command_pub = nh->advertise<mav_msgs::RollPitchYawrateThrust>("roll_pitch_yawrate_thrust_setpoint", 10);

  // init services
  publish_control_server = pnh->advertiseService("publish_control", &VelocityControlNode::publish_control_callback, this);

  return true;
}

bool VelocityControlNode::execute()
{
  if (got_odom && got_twist && should_publish)
  {
    vx_controller->set_target(setpoint_vel_target_frame.x());
    vy_controller->set_target(setpoint_vel_target_frame.y());
    vz_controller->set_target(setpoint_vel_target_frame.z());
    yawrate_controller->set_target(setpoint_ang_vel_target_frame.z());

    tf::Vector3 thrust_des = tf::Vector3(
        vx_controller->get_control(actual_vel_target_frame.x(), 1),
        vy_controller->get_control(actual_vel_target_frame.y(), 1),
        vz_controller->get_control(actual_vel_target_frame.z(), 1));
#ifdef DEBUG
    ROS_ERROR("Target X: %0.1lf, Y: %0.1lf, Z: %0.1lf", setpoint_vel_target_frame.x(), setpoint_vel_target_frame.y(), setpoint_vel_target_frame.z());
    ROS_ERROR("Measure X: %0.1lf, Y: %0.1lf, Z: %0.1lf", actual_vel_target_frame.x(), actual_vel_target_frame.y(), actual_vel_target_frame.z());
    ROS_ERROR("Output X: %0.1lf, Y: %0.1lf, Z: %0.1lf", thrust_des.x(), thrust_des.y(), thrust_des.z());
#endif
    thrust_des.setZ(thrust_des.z() + hover_thrust);

    tf::Quaternion att_sp;
    double total_thrust;
    std::tie(att_sp, total_thrust) = CalculateAttitudeThrust(thrust_des);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf::Matrix3x3(att_sp).getRPY(roll, pitch, yaw);

    float yawrate = yawrate_controller->get_control(actual_ang_vel_target_frame.z(), 1);

    mav_msgs::RollPitchYawrateThrust drone_cmd;
    drone_cmd.roll = roll;
    drone_cmd.pitch = pitch;
    drone_cmd.yaw_rate = yawrate;
    drone_cmd.thrust.z = total_thrust;

    command_pub.publish(drone_cmd);
  }

  return true;
}

void VelocityControlNode::odom_callback(nav_msgs::Odometry odom)
{
  try
  {
    tf::StampedTransform odom_to_target_tf;
    listener->waitForTransform(target_frame, odom.child_frame_id, odom.header.stamp, ros::Duration(0.1));
    listener->lookupTransform(target_frame, odom.child_frame_id, odom.header.stamp, odom_to_target_tf);
    // remove translation, we only care about rotating the velocities
    odom_to_target_tf.setOrigin(tf::Vector3(0, 0, 0));

    tf::Vector3 vel_odom_frame(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
    actual_vel_target_frame = odom_to_target_tf * vel_odom_frame;
    tf::Vector3 ang_vel_odom_frame(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z);
    actual_ang_vel_target_frame = odom_to_target_tf * ang_vel_odom_frame;

    got_odom = true;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

void VelocityControlNode::twist_callback(geometry_msgs::TwistStamped twist)
{
  try
  {
    tf::StampedTransform twist_to_target_tf;
    listener->waitForTransform(target_frame, twist.header.frame_id, twist.header.stamp, ros::Duration(0.1));
    listener->lookupTransform(target_frame, twist.header.frame_id, twist.header.stamp, twist_to_target_tf);
    // remove translation, we only care about rotating the velocities
    twist_to_target_tf.setOrigin(tf::Vector3(0, 0, 0));

    tf::Vector3 vel_twist_frame(twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z);
    setpoint_vel_target_frame = twist_to_target_tf * vel_twist_frame;
    tf::Vector3 ang_vel_twist_frame(twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z);
    setpoint_ang_vel_target_frame = twist_to_target_tf * ang_vel_twist_frame;

    got_twist = true;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}
bool VelocityControlNode::publish_control_callback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response)
{
  // if we transition from not publishing to publishing, reset the integrators
  if (!should_publish && request.data)
  {
    vx_controller->reset_integral();
    vy_controller->reset_integral();
    vz_controller->reset_integral();
    yawrate_controller->reset_integral();
  }

  should_publish = request.data;

  response.success = true;
  return true;
}

void VelocityControlNode::mute_callback(const std_msgs::Bool &mute)
{
  ROS_WARN("here in vel mute callback");
  if (!should_publish && mute.data)
  {
    vx_controller->reset_integral();
    vy_controller->reset_integral();
    vz_controller->reset_integral();
    yawrate_controller->reset_integral();
  }
  should_publish = mute.data;
}

std::tuple<tf::Quaternion, double> VelocityControlNode::CalculateAttitudeThrust(tf::Vector3 thrust_sp)
{
  // Convert ENU to NED for convenience
  // float yaw_ned = M_PI - yaw_sp;
  // tf::Vector3 thrust_ned(thrust_sp.y(), thrust_sp.x(), -thrust_sp.z());

  // Define the body_z axis as the opposite of the thrust direction (i.e., pointing to body bottom)
  // If the thrust is zero, set the body_z to point directly down
  tf::Vector3 body_z(thrust_sp);
  if (body_z.length2() < 1e-6)
    body_z.setZ(1.f);
  body_z.normalize();

  // Get the vector of desired yaw direction in inertial XY plane, rotated by PI/2 (i.e., pointing to the body right on horizontal plane)
  tf::Vector3 y_c(-std::sin(0), std::cos(0), 0.0f);

  // Get the desired body_x axis, orthogonal to body_z and the y_c vector
  tf::Vector3 body_x = y_c.cross(body_z);

  // Make sure the nose is pointing to the front when the UAV is inverted upside down
  if (body_z.z() < 0.0f)
    body_x = -body_x;

  // If the desired thrust is in inertial XY plane, set X downside to construct correct matrix,
  // but yaw component will not be used in this case
  if (fabsf(body_z.z()) < 1e-6f)
  {
    body_x.setZero();
    body_x.setZ(1.0f);
  }

  // Make sure the body_x axis is normalized
  body_x.normalize();

  // Get the desired body_y axis
  tf::Vector3 body_y = body_z.cross(body_x);

  // Construct the rotation matrix from the axes
  tf::Matrix3x3 R_sp(1, 0, 0, 0, 1, 0, 0, 0, 1);
  for (int i = 0; i < 3; i++)
  {
    R_sp[i][0] = body_x[i];
    R_sp[i][1] = body_y[i];
    R_sp[i][2] = body_z[i];
  }

  // Convert the rotation matrix to quaternion
  tf::Quaternion att_sp;
  R_sp.getRotation(att_sp);

  double total_thrust = thrust_sp.length();

  // check total_thrust NaN
  if (total_thrust != total_thrust)
  {
    total_thrust = 0.0;
  }

#ifdef DEBUG
  ROS_ERROR("Thrust X: %0.2lf, Y: %0.2lf, Z: %0.2lf", thrust_sp.x(), thrust_sp.y(), thrust_sp.z());
  double roll, pitch, yaw;
  tf::Matrix3x3(att_sp).getRPY(roll, pitch, yaw);
  ROS_ERROR("Target Roll: %0.2lf, Pitch: %0.2lf, Yaw: %0.2lf, Thrust: %.02lf", roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI, total_thrust);
  ROS_WARN("------");
#endif
  return std::make_tuple(att_sp, total_thrust);
}

VelocityControlNode::~VelocityControlNode()
{
}

BaseNode *BaseNode::get()
{
  VelocityControlNode *velocity_control_node = new VelocityControlNode();
  return velocity_control_node;
}
