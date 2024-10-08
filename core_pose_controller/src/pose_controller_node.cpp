#include <pose_controller/pose_controller_node.h>
#include <mav_msgs/AttitudeThrust.h>
#include <tuple>

PoseControlNode::PoseControlNode(std::string node_name)
  : BaseNode(node_name) { }

bool PoseControlNode::initialize()
{
  ros::NodeHandle* nh = get_node_handle();
  ros::NodeHandle* pnh = get_private_node_handle();

  // Initialize the fence parameters
  _pos_fence.Enabled = pnh->param("geofence/enable", bool(false));
  _pos_fence.XMin = pnh->param("geofence/x_min", double(_pos_fence.XMin));
  _pos_fence.XMax = pnh->param("geofence/x_max", double(_pos_fence.XMax));
  _pos_fence.YMin = pnh->param("geofence/y_min", double(_pos_fence.YMin));
  _pos_fence.YMax = pnh->param("geofence/y_max", double(_pos_fence.YMax));
  _pos_fence.ZMin = pnh->param("geofence/z_min", double(_pos_fence.ZMin));
  _pos_fence.ZMax = pnh->param("geofence/z_max", double(_pos_fence.ZMax));

  // Initialize other parameters
  _target_frame_str = pnh->param("target_frame", std::string("map"));
  _xy_vel_limit   = pnh->param("max_xy_vel", double(12.f));
  _hover_thrust   = pnh->param("hover_thrust", double(0.5f));
  _thrust_max     = pnh->param("thrust_max", double(1.0f));
  _thrust_min     = pnh->param("thrust_min", double(0.15f));
  _max_tilt_deg   = pnh->param("max_tilt", double(45.0f));
  _execute_target = pnh->param("execute_target", double(0.0f));


  // Create the new pose controller
  _pose_controller = new PoseController(_pos_fence, _target_frame_str, _xy_vel_limit,
      _thrust_min, _thrust_max, _max_tilt_deg, _hover_thrust);

  // Initialize the variables


  // Initialize subscribers
  _tracking_point_sub = nh->subscribe("tracking_point", 10, &PoseControlNode::_tracking_point_callback, this);
  _arm_sub = nh->subscribe("arm_active", 10, &PoseControlNode::_arm_callback, this);
  _odometry_sub = nh->subscribe("odometry", 10, &PoseControlNode::_odometry_callback, this);
  _mute_sub = nh->subscribe("pose_controller/mute_control", 10, &PoseControlNode::_mute_callback, this);
  _tf_listener = new tf::TransformListener();

  // Initialize publishers
  _command_pub = nh->advertise<mav_msgs::AttitudeThrust>("attitude_thrust_command", 10);


  // Print some useful information
  if (_pos_fence.Enabled)
    ROS_INFO("Position geofence is enabled with: x: [%0.1lf, %0.1lf]  y: [%0.1lf, %0.1lf]  z: [%0.1lf, %0.1lf].",
      _pos_fence.XMin, _pos_fence.XMax, _pos_fence.YMin, _pos_fence.YMax, _pos_fence.ZMin, _pos_fence.ZMax);
  else
    ROS_WARN("Geofence is disabled.");
  ROS_INFO("Subscribed to '%s' for odometry data.", _odometry_sub.getTopic().c_str());
  ROS_INFO("Subscribed to '%s' for pose setpoints.", _tracking_point_sub.getTopic().c_str());
  ROS_INFO("Controller loop running with %0.1lf Hz.", _execute_target);

  return true;
}

bool PoseControlNode::execute()
{
  tf::Vector3 thrust_des;
  if (_pose_controller->CalculateThrust(thrust_des) == false)
    return true;

  // Calculate the desired attitude from the thrust
  tf::Quaternion att_sp;
  double total_thrust;
  std::tie(att_sp, total_thrust) = _pose_controller->CalculateAttitudeThrust(thrust_des);

  // Publish the desired thrusts
  mav_msgs::AttitudeThrust drone_cmd;
  drone_cmd.attitude.x = att_sp.getX();
  drone_cmd.attitude.y = att_sp.getY();
  drone_cmd.attitude.z = att_sp.getZ();
  drone_cmd.attitude.w = att_sp.getW();
  drone_cmd.thrust.x = 0;
  drone_cmd.thrust.y = 0;
  drone_cmd.thrust.z = total_thrust;
  // ROS_ERROR("mute = %d", _pose_controller -> mute);
  if(!_pose_controller->mute)
  {
    _command_pub.publish(drone_cmd);
  }
  
  
  
  return true;
}

void PoseControlNode::_tracking_point_callback(const nav_msgs::Odometry &tracking_point)
{
  _pose_controller->UpdateTarget(tracking_point, _tf_listener);
}

void PoseControlNode::_arm_callback(const std_msgs::Bool &armed)
{
  if(armed.data == true)
  {
    _pose_controller->Reset();
  }
}

void PoseControlNode::_odometry_callback(const nav_msgs::Odometry &odom)
{
  _pose_controller->UpdateState(odom, _tf_listener);
}

void PoseControlNode::_mute_callback(const std_msgs::Bool &mute)
{
  ROS_WARN("here in mute callback");
  if(_pose_controller -> mute != mute.data) 
  {
    _pose_controller = new PoseController(_pos_fence, _target_frame_str, _xy_vel_limit,
      _thrust_min, _thrust_max, _max_tilt_deg, _hover_thrust);
    _pose_controller -> mute = mute.data;
    _pose_controller -> Reset();
  }
}



PoseControlNode::~PoseControlNode() { }

BaseNode* BaseNode::get()
{
  PoseControlNode* pose_control_node = new PoseControlNode("pose_controller");
  return pose_control_node;
}
