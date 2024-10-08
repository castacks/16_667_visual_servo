#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <core_drone_interface/DroneCommand.h>

bool is_offboard = false;
bool is_armed = false;

void mode_cb(const std_msgs::Bool::ConstPtr &msg)
{
    if (is_offboard && !msg->data)
        ROS_INFO("Offboard disabled");
    is_offboard = msg->data;
}

void arm_cb(const std_msgs::Bool::ConstPtr &msg)
{
    is_armed = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_offboard_example_node");
    ros::NodeHandle nh;

    ros::Subscriber mode_sub = nh.subscribe<std_msgs::Bool>("has_control", 10, mode_cb);
    ros::Subscriber arm_sub = nh.subscribe<std_msgs::Bool>("is_armed", 10, arm_cb);
    ros::Publisher pose_command_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_command", 10);
    ros::ServiceClient drone_command_client = nh.serviceClient<core_drone_interface::DroneCommand>("drone_command");

    //the setpoint publishing rate MUST be faster than 2Hz
    // PX4 has a timeout of 500ms between two Offboard commands. If this timeout is exceeded, the commander will 
    // fall back to the last mode the vehicle was in before entering Offboard mode. This is why the publishing 
    // rate must be faster than 2 Hz to also account for possible latencies. This is also the same reason why 
    // it is recommended to enter Offboard mode from Position mode, this way if the vehicle drops out of Offboard 
    // mode it will stop in its tracks and hover.
    ros::Rate rate(20.0);

    // Prepare the set_offboard_mode so it switches the mode to offboard mode when called
    core_drone_interface::DroneCommand set_offboard_mode;
    set_offboard_mode.request.command = core_drone_interface::DroneCommandRequest::REQUEST_CONTROL;

    // Prepare the arm_command so it arms the vehicle when called
    core_drone_interface::DroneCommand arm_command;
    arm_command.request.command = core_drone_interface::DroneCommandRequest::ARM;

    ros::Time last_request = ros::Time::now();

    bool pose_published = false; // Trying to publish the position only once to see how it works
    while (ros::ok())
    {
        if (!is_offboard &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (drone_command_client.call(set_offboard_mode) &&
                set_offboard_mode.response.success)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!is_armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (drone_command_client.call(arm_command) &&
                    arm_command.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // Publish the position only once
        if (!pose_published && is_offboard)
        {
            // Even though the PX4 Pro Flight Stack operates in the aerospace NED coordinate frame, 
            // MAVROS translates these coordinates to the standard ENU frame and vice-versa. This is why 
            // we set z to positive 2.
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = 10;
            pose.pose.position.y = 0;
            pose.pose.position.z = 2;
            pose_command_pub.publish(pose);
            pose_published = true;
            ROS_INFO("Pose published.");
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
