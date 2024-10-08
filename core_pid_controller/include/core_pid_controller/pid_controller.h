#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include <ros/ros.h>

#include <core_pid_controller/PIDConfig.h>
#include <core_pid_controller/PIDInfo.h>
#include <std_srvs/Empty.h>

class PIDController {
private:
  std::string node_namespace;
  ros::Publisher pub_pid_info;

  ros::ServiceServer reset_integrator_server;

  double (*calculate_error_func)(double, double);

  bool active;

  double P, I, D, FF;
  bool use_negative_gains;
  double neg_P, neg_I, neg_D, neg_FF;
  double integral, derivative, derivative_filtered;
  double error_prev;

  double integral_threshold;
  double minimum, maximum;
  double constant;

  double target;

  ros::Time time_prev;

  bool reset_integrator(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
public:
  /*PIDController(std::string node_namespace_="~", double p=0, double i=0, double d=0, double ff=0,
		double minimum_=std::numeric_limits<double>::min(), double maximum_=std::numeric_limits<double>::max());
  */
  PIDController(std::string name);
  
  core_pid_controller::PIDInfo pid_info_msg;


  void set_P(double p);
  void set_I(double i);
  void set_D(double d);
  void set_FF(double ff);

  void set_integral_threshold(double integral_thresold);
  void set_minimum(double minimum_);
  void set_maximum(double maximum_);
  void set_constant(double constant_);

  void set_calculate_error_func(double (*func)(double, double));

  void set_target(double target);
  double get_control(double actual, double ff_quantity=0.f);

  void reset_integral();

  void dynamic_reconfigure_callback(core_pid_controller::PIDConfig& config, uint32_t level);
};

double calculate_error_minus(double target, double actual);
double calculate_error_angle(double target, double actual);


#endif
