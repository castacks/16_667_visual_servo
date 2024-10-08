#include <pose_controller/pose_controller.h>
#include <ros/console.h>
#include <cmath>
#include <limits>

// #define DEBUG

GeoFence::GeoFence()
  : Enabled(false), XMin(std::numeric_limits<double>::min()), XMax(std::numeric_limits<double>::max()), 
    YMin(std::numeric_limits<double>::min()), YMax(std::numeric_limits<double>::max()), 
    ZMin(std::numeric_limits<double>::min()), ZMax(std::numeric_limits<double>::max()) 
    {}

GeoFence::GeoFence(bool enabled, double x_min, double x_max, 
  double y_min, double y_max, double z_min, double z_max)
  : Enabled(enabled), XMin(x_min), XMax(x_max), YMin(y_min), 
    YMax(y_max), ZMin(z_min), ZMax(z_max) {}

bool GeoFence::Check(const tf::Vector3 &curr_pose, tf::Vector3 &out_pose)
{
  out_pose.setValue(curr_pose.x(), curr_pose.y(), curr_pose.z());
  if (Enabled == false) 
    return true;
  bool modified = false;
  if (out_pose.x() > XMax) { out_pose.setX(XMax); modified = true; }
  else if (out_pose.x() < XMin) { out_pose.setX(XMin); modified = true; }
  if (out_pose.y() > YMax) { out_pose.setY(YMax); modified = true; }
  else if (out_pose.y() < YMin) { out_pose.setY(YMin); modified = true; }
  if (out_pose.z() > ZMax) { out_pose.setZ(ZMax); modified = true; }
  else if (out_pose.z() < ZMin) { out_pose.setZ(ZMin); modified = true; }
  return !modified;
}

PoseController::PoseController(const GeoFence &pos_fence, const std::string &target_frame, 
      double xy_vel_lim, double thrust_min, double thrust_max, 
      double max_tilt_deg, double hover_thrust, double hover_thrust_filter_gain) :
          PositionFence(pos_fence),
          _target_frame_str(target_frame), _xy_vel_limit(xy_vel_lim),
          _hover_thrust(hover_thrust), _thrust_max(thrust_max),
          _thrust_min(thrust_min), _max_tilt(max_tilt_deg * M_PI / 180.0),
          _last_thrust(hover_thrust), _got_odometry(false), _got_tracking_point(false), 
          _got_visual_measure(false), _got_visual_target(false), _hover_thrust_origin(hover_thrust),
          _hover_thrust_filter_start(false), _hover_thrust_filtered(hover_thrust), _hover_thrust_filter_gain(hover_thrust_filter_gain)
{
  _x_controller = new PIDController("~/x");
  _y_controller = new PIDController("~/y");
  _z_controller = new PIDController("~/z");
  _vx_controller = new PIDController("~/vx");
  _vy_controller = new PIDController("~/vy");
  _vz_controller = new PIDController("~/vz");

  _yaw_controller = new PIDController("~/yaw");

  _visual_x_controller = new PIDController("~/visual_x");
  _visual_z_controller = new PIDController("~/visual_z");
  _visual_y_controller = new PIDController("~/visual_y");

  _visual_vz_estimator = new PIDController("~/visual_vz_est");

  _visual_vx_controller = new PIDController("~/visual_vx");
  _visual_vz_controller = new PIDController("~/visual_vz");
  _visual_vy_controller = new PIDController("~/visual_vy");

  mute = false;

}

void PoseController::UpdateTarget(const nav_msgs::Odometry &target, const tf::TransformListener* tf_listener)
{
  // Transform the tracking point to the target frame
  tf::StampedTransform tracking_point_pos_to_target_tf, tracking_point_vel_to_target_tf;
  try
  {
    _tracking_point = target;

    // Get transforms for position and velocity of the tracking point to the target frame
    tf_listener->waitForTransform(_target_frame_str, target.header.frame_id,
        target.header.stamp, ros::Duration(0.1));
    tf_listener->lookupTransform(_target_frame_str, target.header.frame_id,
        target.header.stamp, tracking_point_pos_to_target_tf);

    tf_listener->waitForTransform(_target_frame_str, target.child_frame_id,
        target.header.stamp, ros::Duration(0.1));
    tf_listener->lookupTransform(_target_frame_str, target.child_frame_id,
        target.header.stamp, tracking_point_vel_to_target_tf);

    // Set velocity transform translation to zero (only use rotation for velocity)
    tracking_point_vel_to_target_tf.setOrigin(tf::Vector3(0, 0, 0));

    // Transform position to the target frame
    _tracking_point_pos_target_frame = tracking_point_pos_to_target_tf * tf::Vector3(
        target.pose.pose.position.x,
				target.pose.pose.position.y,
				target.pose.pose.position.z);

    // Check if the position command is within the fence
    tf::Vector3 modified_tracking_pos;
    if (PositionFence.Check(_tracking_point_pos_target_frame, modified_tracking_pos) == false)
    {
      ROS_WARN_THROTTLE(2, "Fence Breach: Position setpoint changed from (%0.1lf, %0.1lf, %0.1lf) to (%0.1lf, %0.1lf, %0.1lf). This warning is printed at most once every 2 seconds.",
        _tracking_point_pos_target_frame.x(), _tracking_point_pos_target_frame.y(), _tracking_point_pos_target_frame.z(), 
        modified_tracking_pos.x(), modified_tracking_pos.y(), modified_tracking_pos.z());
      _tracking_point_pos_target_frame.setValue(modified_tracking_pos.x(), modified_tracking_pos.y(), modified_tracking_pos.z());
    }

    // Transform velocity to the target frame
    _tracking_point_vel_target_frame = tracking_point_vel_to_target_tf * tf::Vector3(
        target.twist.twist.linear.x,
        target.twist.twist.linear.y,
        target.twist.twist.linear.z);
    // Transform yaw to target frame
    _tracking_point_yaw_target_frame = tf::getYaw(tracking_point_pos_to_target_tf * tf::Quaternion(
        target.pose.pose.orientation.x,
        target.pose.pose.orientation.y,
        target.pose.pose.orientation.z,
        target.pose.pose.orientation.w));

    if (_got_tracking_point == false)
    {
      ROS_INFO("Successfully received a pose tracking point. This message will not be published again.");
      _got_tracking_point = true;
    }
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR_THROTTLE(1, "Tracking point TF lookup failed: %s", ex.what());
  }
}

void PoseController::UpdateState(const nav_msgs::Odometry &odom, const tf::TransformListener *tf_listener)
{
  // Transform the state estimate odometry to the target frame
  tf::StampedTransform odom_pos_to_target_tf, odom_vel_to_target_tf;
  try
  {
    _odometry = odom;
    
    // Get transforms for position and velocity of the tracking point to the target frame
    tf_listener->waitForTransform(_target_frame_str, odom.header.frame_id,
        odom.header.stamp, ros::Duration(0.1));
    tf_listener->lookupTransform(_target_frame_str, odom.header.frame_id,
        odom.header.stamp, odom_pos_to_target_tf);

    tf_listener->waitForTransform(_target_frame_str, odom.child_frame_id,
        odom.header.stamp, ros::Duration(0.1));
    tf_listener->lookupTransform(_target_frame_str, odom.child_frame_id,
        odom.header.stamp, odom_vel_to_target_tf);

    // Set velocity transform translation to zero (only use rotation for velocity)
    odom_vel_to_target_tf.setOrigin(tf::Vector3(0, 0, 0));
    
    // Transform position to the target frame
    _odometry_pos_target_frame = odom_pos_to_target_tf * tf::Vector3(
        odom.pose.pose.position.x,
        odom.pose.pose.position.y,
        odom.pose.pose.position.z);

    // Transform velocity to the target frame
    _odometry_vel_target_frame = odom_vel_to_target_tf * tf::Vector3(
        odom.twist.twist.linear.x, 
        odom.twist.twist.linear.y,
        odom.twist.twist.linear.z);

    if (_got_odometry == false)
    {
      ROS_INFO("Successfully received odometry data. This message will not be published again.");
      _got_odometry = true;
    }
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR_THROTTLE(1, "Odometry TF lookup failed: %s", ex.what());
  }
}

void PoseController::UpdateStateVisual(const geometry_msgs::PoseStamped &measure)
{
  if (_got_visual_measure == false)
  {
    ROS_INFO("Successfully received visual data. This message will not be published again.");
    _got_visual_measure = true;
  }

  _visual_measure = measure;
}

void PoseController::UpdateTargetVisual(const geometry_msgs::PoseStamped &target)
{
  if (_got_visual_target == false)
  {
    ROS_INFO("Successfully received visual targte data. This message will not be published again.");
    _got_visual_target = true;
  }

  _visual_target = target;
}

bool PoseController::CalculateThrustVisual(tf::Vector3 &thrust_des)
{
  // Check if we have received all the required data and should publish
  if (!_got_visual_measure || !_got_visual_target)
  {
    if (_got_visual_measure == false && _got_visual_target == false)
      ROS_WARN_THROTTLE(5, "Waiting for the visual meausre data and visual target data...");
    else if (_got_visual_measure == false)
      ROS_WARN_THROTTLE(5, "Waiting for the visual measure data...");
    else
      ROS_WARN_THROTTLE(5, "Waiting for the visual target data...");
    return false;
  }

  // Check if the visual data is too old
  if ((ros::Time::now() - _visual_measure.header.stamp).toSec() > 2.0)
  {
    ROS_ERROR_THROTTLE(5, "ERROR: Visual data is too old. Missing visual measure, switch back to pose control");
    return false;
  }
  // ------- Visual Z Control -------------
  double gain=1.0;

  

  _visual_vz_estimator->set_target(_visual_measure.pose.position.y);
  double estimated_Z_image_velocity = _visual_vz_estimator->get_control(0, 0.0)*_visual_measure.pose.position.z;

  // visual_z == depth visual_y == image_y
  _visual_x_controller->set_target(_tracking_point_pos_target_frame.x());
  _visual_y_controller->set_target(_visual_target.pose.position.x*_visual_measure.pose.position.z*gain);
  _visual_z_controller->set_target(_visual_target.pose.position.y*_visual_measure.pose.position.z*gain);

  // Get the desired velocities
  tf::Vector3 vel_des(
      _visual_x_controller->get_control(_odometry_pos_target_frame.x(), 0),
      _visual_y_controller->get_control(_visual_measure.pose.position.x*_visual_measure.pose.position.z*gain, 0),
      _visual_z_controller->get_control(_visual_measure.pose.position.y*_visual_measure.pose.position.z*gain, 0));

  // Constrain the xy velocities
  tf::Vector3 vel_h_lim = _constrain_velocity(vel_des, _tracking_point_vel_target_frame);
  vel_des.setX(vel_h_lim.x());
  vel_des.setY(vel_h_lim.y());
  vel_des.setZ(vel_des.z());

  // ------- Velocity Control -------------

  // Set the target for the velocity controllers
  _visual_vx_controller->set_target(vel_des.x());
  _visual_vy_controller->set_target(vel_des.y());
  _visual_vz_controller->set_target(vel_des.z());

  // Get the desired thrusts
  thrust_des = tf::Vector3(
      _visual_vx_controller->get_control(_odometry_vel_target_frame.x(), 0),
      _visual_vy_controller->get_control(_odometry_vel_target_frame.y(), 0),
      _visual_vz_controller->get_control(_odometry_vel_target_frame.z(), 0));
  thrust_des.setZ(thrust_des.z() + _hover_thrust);



  return true;
}

bool PoseController::CalculateYawRate(double &yaw_rate_des)
{
  // Check if we have received all the required data and should publish
  if (!_got_visual_measure)
  {  
    ROS_WARN_THROTTLE(5, "Waiting for the visual measure...");
    return false;
  }

  // Check if the position data is too old
  if ((ros::Time::now() - _visual_measure.header.stamp).toSec() > 2.0)
    ROS_WARN_THROTTLE(2, "Warning: visual measure is too old.");

  // ------- Yaw Rate Control -------------
  double r=0.0,p=0.0,y=0.0;
  tf::Quaternion q;
  tf::quaternionMsgToTF(_visual_measure.pose.orientation,q);
  tf::Matrix3x3(q).getRPY(r, p, y);
  _yaw_controller->set_target(0);
  yaw_rate_des = _yaw_controller->get_control(y, 0);

  ROS_DEBUG_THROTTLE(2, "yaw_target: %.2f, yaw_measure: %.2f, yaw_rate: %.2f", 0.0f,y,yaw_rate_des);

  return true;
}

bool PoseController::CalculateThrust(tf::Vector3 &thrust_des)
{
  // Check if we have received all the required data and should publish
  if (!_got_odometry || !_got_tracking_point)
  {
    if (_got_odometry == false && _got_tracking_point == false)
      ROS_WARN_THROTTLE(5, "Waiting for the odometry data and pose tracking point...");
    else if (_got_odometry == false)
      ROS_WARN_THROTTLE(5, "Waiting for the odometry data...");
    else
      ROS_WARN_THROTTLE(5, "Waiting for the pose tracking point...");
    return false;
  }

  // Check if the position data is too old
  if ((ros::Time::now() - _odometry.header.stamp).toSec() > 2.0)
    ROS_WARN_THROTTLE(2, "Warning: Odometry data is too old.");

  // ------- Position Control -------------

  // Set the target for position controllers
  _x_controller->set_target(_tracking_point_pos_target_frame.x());
  _y_controller->set_target(_tracking_point_pos_target_frame.y());
  _z_controller->set_target(_tracking_point_pos_target_frame.z());

  // Get the desired velocities
  tf::Vector3 vel_des(
      _x_controller->get_control(_odometry_pos_target_frame.x(), 0),
      _y_controller->get_control(_odometry_pos_target_frame.y(), 0),
      _z_controller->get_control(_odometry_pos_target_frame.z(), 0));

  // Constrain the xy velocities
  tf::Vector3 vel_h_lim = _constrain_xy_velocity(vel_des, _tracking_point_vel_target_frame);
  vel_des.setX(vel_h_lim.x());
  vel_des.setY(vel_h_lim.y());
  vel_des.setZ(vel_des.z() + _tracking_point_vel_target_frame.z());

  // ------- Velocity Control -------------

  // Set the target for the velocity controllers
  _vx_controller->set_target(vel_des.x());
  _vy_controller->set_target(vel_des.y());
  _vz_controller->set_target(vel_des.z());

  // Get the desired thrusts
  thrust_des = tf::Vector3(
      _vx_controller->get_control(_odometry_vel_target_frame.x(), 0),
      _vy_controller->get_control(_odometry_vel_target_frame.y(), 0),
      _vz_controller->get_control(_odometry_vel_target_frame.z(), 0));
  
  thrust_des.setZ(thrust_des.z() + _hover_thrust);



  return true;
}

void PoseController::ConstrainThrust(tf::Vector3 & thrust_des)
{
  // Set the Z thrust to the limits
  thrust_des.setZ(std::max(std::min(thrust_des.z(), _thrust_max), _thrust_min));

  // Enforce the maximum tilt and thrust
  tf::Vector3 thrust_h_des = ConstrainHorizontalThrust(thrust_des);
  thrust_des.setX(thrust_h_des.x());
  thrust_des.setY(thrust_h_des.y());
}

std::tuple<tf::Quaternion, double> PoseController::CalculateAttitudeThrust(tf::Vector3 thrust_sp)
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
	tf::Vector3 y_c(-std::sin(_tracking_point_yaw_target_frame), std::cos(_tracking_point_yaw_target_frame), 0.0f);

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
  if(total_thrust != total_thrust) 
  {
    total_thrust=0.0;
  }

  // Limit the thrust rate
  // TODO: Improve it
  double dt = 0.02;
  double thrust_rate = 5;
  if (std::abs(_last_thrust - total_thrust) > thrust_rate * dt)
    total_thrust = _last_thrust + (total_thrust - _last_thrust) * thrust_rate * dt;


  _last_thrust = total_thrust;

  #ifdef DEBUG
    double roll, pitch, yaw;
    tf::Quaternion curr_q(_odometry.pose.pose.orientation.x, _odometry.pose.pose.orientation.y, _odometry.pose.pose.orientation.z, _odometry.pose.pose.orientation.w);
    tf::Matrix3x3(curr_q).getRPY(roll, pitch, yaw);
    ROS_ERROR("Current Roll: %0.1lf, Pitch: %0.1lf, Yaw: %0.1lf", roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);
    tf::Matrix3x3(att_sp).getRPY(roll, pitch, yaw);
    ROS_ERROR("Target Roll: %0.1lf, Pitch: %0.1lf, Yaw: %0.1lf", roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);
    ROS_WARN("------");
  #endif

  return std::make_tuple(att_sp, total_thrust);
}

void PoseController::UpdateZThrustFilter(const tf::Vector3 thrust)
{
  double z_thrust = thrust.z();
  if(_hover_thrust_filter_start)
  {
    _hover_thrust_filtered = _hover_thrust_filtered * (1.0-_hover_thrust_filter_gain) + z_thrust * (_hover_thrust_filter_gain);
  }
  else
  {
    _hover_thrust_filter_start = true;
    _hover_thrust_filtered = _hover_thrust_origin;
  }
}

void PoseController::UpdateHoverThrust(void)
{
  if(_hover_thrust_filter_start)
  {
    _hover_thrust = _hover_thrust_filtered;
  }
  else
  {
    _hover_thrust = _hover_thrust_origin;
  }
  _hover_thrust_filter_start=false;
}

void PoseController::GetHoverThrust(double &hover_thrust_filterd, double &hover_thrust, double &hover_thrust_origin)
{
  hover_thrust_filterd = _hover_thrust_filtered;
  hover_thrust = _hover_thrust;
  hover_thrust_origin = _hover_thrust_origin;
}


tf::Vector3 PoseController::_constrain_xy_velocity(const tf::Vector3 &v0, const tf::Vector3 &v1)
{
  // Method is similar to PX4

  tf::Vector3 v0_h(v0.x(), v0.y(), 0.0f);
  tf::Vector3 v1_h(v1.x(), v1.y(), 0.0f);

  tf::Vector3 res = v0_h + v1_h;

  if (res.length2() <= _xy_vel_limit * _xy_vel_limit)
    return res;

  if ((v0_h.length2() > _xy_vel_limit * _xy_vel_limit) || ((v0_h - v1_h).length2() < 0.01f))
    return v0_h.normalized() * _xy_vel_limit;
  
  if (v0_h.length2() < 0.01f)
    return v1_h.normalized() * _xy_vel_limit;
  
  v1_h.normalize();
  float m = v1_h.dot(v0_h);
  float c = v0_h.length2() - _xy_vel_limit * _xy_vel_limit;
  float scale = -m + std::sqrt(m * m - c);
  return v0 + v1_h * scale;
}

tf::Vector3 PoseController::_constrain_velocity(const tf::Vector3 &v0, const tf::Vector3 &v1)
{
  // Method is similar to PX4

  tf::Vector3 v0_h(v0.x(), v0.y(), 0.0f);
  tf::Vector3 v1_h(v1.x(), v1.y(), 0.0f);

  tf::Vector3 res = v0_h + v1_h;

  double _vel_limit = _xy_vel_limit/1.414;

  res.setX(std::max(std::min(res.x(), _vel_limit), -_vel_limit));
  res.setY(std::max(std::min(res.y(), _vel_limit), -_vel_limit));

  return res;

}

tf::Vector3 PoseController::ConstrainHorizontalThrust(const tf::Vector3 &thr)
{
  // Find the maximum thrust for the tilt and thrust constraints
  double max_h_thr_tilt = std::abs(thr.z()) * std::tan(_max_tilt);
  double max_h_thr_lim = std::sqrt(_thrust_max * _thrust_max - thr.z() * thr.z());
	double max_h_thr = std::min(max_h_thr_lim, max_h_thr_tilt);

  tf::Vector3 thr_h(thr.x(), thr.y(), 0.0f);
 
  if (thr_h.length2() > max_h_thr * max_h_thr) 
  {
    float mag = thr_h.length();
    thr_h.setX(thr_h.x() / mag * max_h_thr);
    thr_h.setY(thr_h.y() / mag * max_h_thr);
  }

  return thr_h;
}

void PoseController::Reset()
{
  _x_controller->reset_integral();
  _y_controller->reset_integral();
  _z_controller->reset_integral();
  _vx_controller->reset_integral();
  _vy_controller->reset_integral();
  _vz_controller->reset_integral();

  _visual_z_controller->reset_integral();
  // _visual_y_controller->reset_integral();
}

