// #include <core_pid_controller/pid_controller_matrix.hpp>
#include <ros/ros.h>
#include <base/BaseNode.h>
#include <core_pid_controller/pid_controller.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_listener.h>
#include <mav_msgs/RollPitchYawrateThrust.h>

#include <iostream>
#include <cstdio>

#define DEBUG

std::tuple<tf::Quaternion, double> CalculateAttitudeThrust(tf::Vector3 thrust_sp)
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
  if(total_thrust != total_thrust) 
  {
    total_thrust=0.0;
  }


  #ifdef DEBUG
    ROS_ERROR("Thrust X: %0.2lf, Y: %0.2lf, Z: %0.2lf", thrust_sp.x(),thrust_sp.y(),thrust_sp.z());
    double roll, pitch, yaw;
    tf::Matrix3x3(att_sp).getRPY(roll, pitch, yaw);
    ROS_ERROR("Target Roll: %0.2lf, Pitch: %0.2lf, Yaw: %0.2lf", roll * 180 / M_PI, pitch * 180 / M_PI, yaw * 180 / M_PI);
    ROS_WARN("------");
  #endif

  return std::make_tuple(att_sp, total_thrust);
}

int main(void)
{
    // test_pinv();

    tf::Vector3 thrust_des = tf::Vector3(0.0,0.1,0.55);

    tf::Quaternion att_sp;
    double total_thrust;
    std::tie(att_sp, total_thrust) = CalculateAttitudeThrust(thrust_des);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf::Matrix3x3(att_sp).getRPY(roll, pitch, yaw);

    ROS_ERROR("No Hover Thrust X: %0.2lf, Y: %0.2lf, Z: %0.2lf", thrust_des.x(),thrust_des.y(),thrust_des.z());

    return 0;
}