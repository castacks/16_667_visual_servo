#ifndef _POSE_CONTROLLER_H_
#define _POSE_CONTROLLER_H_

#include <core_pid_controller/pid_controller.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <tuple>

class GeoFence
{
public:
  GeoFence();

  GeoFence(bool enabled, double x_min, double x_max, 
    double y_min, double y_max, double z_min, double z_max);

  bool Check(const tf::Vector3 &curr_pose, tf::Vector3 &out_pose);

  bool Enabled;
  double XMin, XMax, YMin, YMax, ZMin, ZMax;
};

class PoseController
{
private:
  // Controllers
  PIDController* _x_controller;
  PIDController* _y_controller;
  PIDController* _z_controller;
  PIDController* _vx_controller;
  PIDController* _vy_controller;
  PIDController* _vz_controller;

  PIDController* _yaw_controller;
  PIDController* _visual_x_controller;
  PIDController* _visual_vx_controller;
  PIDController* _visual_y_controller;
  PIDController* _visual_vy_controller;
  PIDController* _visual_z_controller;
  PIDController* _visual_vz_estimator;
  PIDController* _visual_vz_controller;

  


  // Variables
  std::string _target_frame_str;
  double _xy_vel_limit, _hover_thrust, _thrust_min, _thrust_max, _max_tilt, _last_thrust;
  double _hover_thrust_origin, _hover_thrust_filter_gain, _hover_thrust_filtered;
  bool _track_velocities, _got_tracking_point, _got_odometry, _got_visual_measure, _got_visual_target, _hover_thrust_filter_start;
  
  // Latest tracking point and odometry readings in original and target frames
  nav_msgs::Odometry _tracking_point, _odometry;
  tf::Vector3 _tracking_point_pos_target_frame, _tracking_point_vel_target_frame;
  tf::Vector3 _odometry_pos_target_frame, _odometry_vel_target_frame;
  double _tracking_point_yaw_target_frame, _odometry_yaw_target_frame;

  geometry_msgs::PoseStamped _visual_target, _visual_measure;

  // Helper functions
  tf::Vector3 _constrain_xy_velocity(const tf::Vector3 &v0, const tf::Vector3 &v1);
  tf::Vector3 _constrain_velocity(const tf::Vector3 &v0, const tf::Vector3 &v1);



public:
  GeoFence PositionFence;

  bool mute = false;

  PoseController(const GeoFence &pos_fence, const std::string &target_frame, double xy_vel_lim, 
      double thrust_min, double thrust_max, 
      double max_tilt_deg, double hover_thrust, double hover_thrust_filter_gain=1.0);
  void UpdateTarget(const nav_msgs::Odometry &target, const tf::TransformListener *tf_listener);
  void UpdateState(const nav_msgs::Odometry &odom, const tf::TransformListener *tf_listener);
  bool CalculateThrust(tf::Vector3 &thrust_des);
  std::tuple<tf::Quaternion, double> CalculateAttitudeThrust(tf::Vector3 thrust_sp);
  tf::Vector3 ConstrainHorizontalThrust(const tf::Vector3 &thr);

  void UpdateTargetVisual(const geometry_msgs::PoseStamped &target);
  void UpdateStateVisual(const geometry_msgs::PoseStamped &measure);
  bool CalculateThrustVisual(tf::Vector3 &thrust_des);
  bool CalculateYawRate(double & yaw_rate_des);
  void UpdateZThrustFilter(const tf::Vector3 thrust);
  void UpdateHoverThrust(void);
  void GetHoverThrust(double &hover_thrust_filterd, double &hover_thrust, double &hover_thrust_origin);

  void ConstrainThrust(tf::Vector3 & thrust_des);  
  void Reset();
};


#endif
