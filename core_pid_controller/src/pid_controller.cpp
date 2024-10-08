#include <core_pid_controller/pid_controller.h>

#include <dynamic_reconfigure/server.h>
#include <core_pid_controller/PIDConfig.h>
#include <core_pid_controller/PIDInfo.h>


PIDController::PIDController(std::string name_space){
  node_namespace = name_space;
  ros::NodeHandle nh(name_space);
  
  dynamic_reconfigure::Server<core_pid_controller::PIDConfig>* cfg_server = new dynamic_reconfigure::Server<core_pid_controller::PIDConfig>(nh);
  dynamic_reconfigure::Server<core_pid_controller::PIDConfig>::CallbackType callback_type;
  callback_type = boost::bind(&PIDController::dynamic_reconfigure_callback, this, _1, _2);
  cfg_server->setCallback(callback_type);
  
  pub_pid_info = nh.advertise<core_pid_controller::PIDInfo>("pid_info", 10, false);

  reset_integrator_server = nh.advertiseService("reset_integrator", &PIDController::reset_integrator, this);

  active = false;

  P = nh.param("P", 0.);
  I = nh.param("I", 0.);
  D = nh.param("D", 0.);
  FF = nh.param("FF", 0.);
  integral_threshold = nh.param("integral_threshold", 4.0);


  use_negative_gains = nh.param("use_negative_gains", false);
  neg_P = nh.param("neg_P", P);
  neg_I = nh.param("neg_I", I);
  neg_D = nh.param("neg_D", D);
  neg_FF = nh.param("neg_FF", FF);
  if(!use_negative_gains){
    neg_P = P;
    neg_I = I;
    neg_D = D;
    neg_FF = FF;
  }

  ROS_WARN("Controller Name: %s, P %.2f, I %.2f, D %.2f, I threshold %.2f",name_space.c_str(),P,I,D,integral_threshold);
  minimum = nh.param("min", std::numeric_limits<double>::min());
  maximum = nh.param("max", std::numeric_limits<double>::max());
  constant = nh.param("constant", 0.);

  integral = 0.f;
  derivative = 0.f;
  error_prev = 0.f;
  derivative_filtered = 0.f;

  calculate_error_func = &calculate_error_minus;

  target = 0.f;
  time_prev = ros::Time(0);

  pid_info_msg.P = P;
  pid_info_msg.I = I;
  pid_info_msg.D = D;
  pid_info_msg.FF = FF;
  pid_info_msg.neg_P = neg_P;
  pid_info_msg.neg_I = neg_I;
  pid_info_msg.neg_D = neg_D;
  pid_info_msg.neg_FF = neg_FF;
  pid_info_msg.integral_threshold = integral_threshold;
  pid_info_msg.min = minimum;
  pid_info_msg.max = maximum;
  pid_info_msg.p_component = 0;
  pid_info_msg.i_component = 0;
  pid_info_msg.d_component = 0;
  pid_info_msg.ff_component = 0;
  pid_info_msg.constant = constant;
  pid_info_msg.control = 0;
  pid_info_msg.actual = 0;
  pid_info_msg.target = target;
  pid_info_msg.error = 0;

  core_pid_controller::PIDConfig config;
  config.P = P;
  config.I = I;
  config.D = D;
  config.FF = FF;
  config.use_negative_gains = use_negative_gains;
  config.neg_P = neg_P;
  config.neg_I = neg_I;
  config.neg_D = neg_D;
  config.neg_FF = neg_FF;
  config.constant = constant;
  config.target = target;
  config.integral_threshold = integral_threshold;
  cfg_server->updateConfig(config);
}

void PIDController::set_P(double p){
  P = p;
  pid_info_msg.P = P;
}

void PIDController::set_I(double i){
  I = i;
  pid_info_msg.I = I;
}

void PIDController::set_D(double d){
  D = d;
  pid_info_msg.D = D;
}

void PIDController::set_FF(double ff){
  FF = ff;
  pid_info_msg.FF = FF;
}

void PIDController::set_integral_threshold(double integral_threshold_)
{
  integral_threshold = integral_threshold_;
  pid_info_msg.integral_threshold = integral_threshold;
}

void PIDController::set_minimum(double minimum_){
  minimum = minimum_;
  pid_info_msg.min = minimum;
}

void PIDController::set_maximum(double maximum_){
  maximum = maximum_;
  pid_info_msg.max = maximum;
}

void PIDController::set_constant(double constant_){
  constant = constant_;
}

void PIDController::set_target(double t){
  target = t;
  pid_info_msg.target = target;
}

double PIDController::get_control(double actual, double ff_quantity){
  // check if this is the first time the function has been called, return 0 if so.
  ros::Time time_now = ros::Time::now();
  if(time_prev == ros::Time(0)){
    time_prev = time_now;
    return 0.f;
  }

  // calculate dt
  double dt = (time_now - time_prev).toSec();

  // if the time is messed up, just return the previous control
  if(dt <= 0){
    pub_pid_info.publish(pid_info_msg);
    return pid_info_msg.control;
  }

  double error = (calculate_error_func)(target, actual);
  integral += error*dt;
  if( integral > integral_threshold ){ integral = integral_threshold;}
  else if( integral < -integral_threshold ){ integral = -integral_threshold;}

  if(active)
  {
    derivative = (error - error_prev)/dt;
  }
  else
  {
    derivative = 0.0;
    active = true;
  }

  // set gains based on the sign of the error term
  double p_gain = P;
  double i_gain = I;
  double d_gain = D;
  double ff_gain = FF;
  if(use_negative_gains && error < 0){
    p_gain = neg_P;
    i_gain = neg_I;
    d_gain = neg_D;
    ff_gain = neg_FF;
  }
  
  //derivative filter
  double derivative_diff = derivative - derivative_filtered;
  if(derivative_diff>0.5*maximum){derivative_diff=0.5*maximum;}
  else if(derivative_diff<0.5*minimum){derivative_diff=0.5*minimum;}
  derivative_filtered = derivative_filtered + (1.0 - ff_gain) * derivative_diff;

  // calculate the control signal
  double p_component = p_gain*error;
  double i_component = i_gain*integral;
  double d_component = D*derivative;
  double ff_component = D*derivative_filtered;
  double control = p_component + i_component + ff_component + constant;
  
  // limit control signal
  control = std::max(std::min(control, maximum), minimum);

  // update time and error
  time_prev = time_now;
  error_prev = error;
  
  // publish info
  pid_info_msg.header.stamp = time_now;
  pid_info_msg.p_component = p_component;
  pid_info_msg.i_component = i_component;
  pid_info_msg.d_component = d_component;
  pid_info_msg.ff_component = ff_component;
  pid_info_msg.constant = constant;
  pid_info_msg.control = control;
  pid_info_msg.actual = actual;
  pid_info_msg.target = target;
  pid_info_msg.error = error;
  pub_pid_info.publish(pid_info_msg);

  return control;
}

void PIDController::set_calculate_error_func(double (*func)(double, double)){
  calculate_error_func = func;
}

void PIDController::reset_integral(){
  integral = 0.f;
}

void PIDController::dynamic_reconfigure_callback(core_pid_controller::PIDConfig& config, uint32_t level){
  double I_prev = I;
  
  P = config.P;
  I = config.I;
  D = config.D;
  FF = config.FF;
  
  use_negative_gains = config.use_negative_gains;
  neg_P = config.neg_P;
  neg_I = config.neg_I;
  neg_D = config.neg_D;
  neg_FF = config.neg_FF;
  
  constant = config.constant;
  
  target = config.target;
  
  if(I != I_prev)
    reset_integral();
}


bool PIDController::reset_integrator(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
  reset_integral();
  return true;
}

double calculate_error_minus(double target, double actual){
  return target - actual;
}

double calculate_error_angle(double target, double actual){
  return atan2(sin(target - actual), cos(target - actual));
}
