#ifndef _BEHAVIOR_TREE_H_
#define _BEHAVIOR_TREE_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <behavior_tree_msgs/Status.h>
#include <string>
#include <algorithm>
#include <thread>
#include <mutex>

namespace bt {
  
  class Condition {
  private:
    std_msgs::Bool success;
    
    ros::Publisher success_pub;
    
  public:
    Condition(ros::NodeHandle nh, std::string label);
    void set(bool b);
    void publish();
    bool get();

    static std::string get_published_topic_name(std::string label);
  };
  
  
  class Action {
  private:
    bool active;
    behavior_tree_msgs::Status status;
    
    ros::Subscriber active_sub;
    ros::Publisher status_pub;
    
    void active_callback(std_msgs::Bool msg);
  public:
    Action(ros::NodeHandle nh, std::string label);
    
    static std::string get_published_topic_name(std::string label);
    static std::string get_subscribed_topic_name(std::string label);
    
    void set_success();
    void set_running();
    void set_failure();

    bool is_active();

    bool is_success();
    bool is_running();
    bool is_failure();

    void publish();
  };
  
};


#endif
