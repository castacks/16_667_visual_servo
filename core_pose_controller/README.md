# Pose Controller for PX4 Fully-Actuated UAV Firmware

This repository is a ROS pose (position and yaw) controller for PX4 autopilot. It is mainly written to control our fully-actuated UAV and uses the [core_px4_interface package](https://bitbucket.org/castacks/core_px4_interface/src/contact-inspection/) to communicate with the robot.

## Installation

First of all, make sure you have installed the autopilot compatible with this package from [https://github.com/castacks/PX4-fully-actuated](https://github.com/castacks/PX4-fully-actuated). Don't forget to also add the `PX4PATH` environment variable to your `.bashrc` file per the instructions on that repository.

Then, make sure you have installed and tested the [core_px4_interface package](https://bitbucket.org/castacks/core_px4_interface/src/contact-inspection/
) already.

Finally, confirm you have a MAVROS version compatible with our PX4 (if you have followed the instructions in our autopilot installation, you will have it in your catkin workspace already).

Now clone the repos into the same catkin workspace as the `Firmware` directory (here we assume it's in `~/catkin_ws/src`) and checkout the branches shown in the links:

```
https://bitbucket.org/castacks/core_pid_controller/src/build-warnings/
https://bitbucket.org/castacks/core_pose_controller/src/contact-inspection/
```

Check if your MAVROS is publishing the `tf` for the local position. Run MAVROS:

```
roslaunch mavros px4.launch fcu_url:="udp://:14540@localhost:14580"
```

Now check if the following parameter is `true`:

```
rosparam get /mavros/local_position/tf/send
```

If the parameter is `false`, go to the local position section in the `mavros/mavros/launch/px4_config.yaml` file and set the `send` parameter to true. After this change, the section should look like this:

```
# local_position
local_position:
  frame_id: "map"
  tf:
    send: true
    frame_id: "map"
    child_frame_id: "base_link"
    send_fcu: false
```

Now `catkin build` the workspace (remember to run it in `catkin_ws`). It will build these repos as well as the PX4 repo from the `Firmware` directory. Don't forget to source the `devel/setup.bash` in the open terminals. It's easier if you just close the terminals and open new ones.

## How to use

First, you need to run all the packages mentioned in the [core_px4_interface package](https://bitbucket.org/castacks/core_px4_interface/src/contact-inspection/) instruction (i.e., PX4, MAVROS and core_px4_interface).

Now, in a new terminal, execute the following command to run the pose controller:

```
roslaunch core_pose_controller pose_controller_gazebo.launch
```

In order to switch from the PX4's internal position controller to this external controller, the flight mode should change to Offboard. Once the switch is done, the position commands to this controller can be sent through the `\tracking_point` ROS topic. Note that the commands should follow the following conventions:

- The coordinates frame for the desired position (i.e., `frame_id`) is ENU (East-North-Up) with the local position origin (usually the take-off point). 
- The coordinates frame for the desired velocity (i.e., `child_frame_id`) is also ENU but with the current UAV position as origin. 
- The value for the `Header.frame_id` field should be set to `map`.
- The value for the `child_frame_id` field should be set to the `base_link` value.
- The values for the desired roll and pitch are currently ignored.

## Contact
Azarakhsh Keipour (keipour@gmail.com)
