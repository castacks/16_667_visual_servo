# ROS Interface for PX4 Fully-Actuated UAV Firmware

This repository is a ROS interface for our Fully-Actuated PX4 autopilot firmware. The package provides ROS services and topics to control the UAV running the autopilot. This interface uses MAVROS to interact with the UAV, but provides convenient methods to change the full-actuated-specific parameters and to send different desired signals (i.e., pose, velocity, thrust/attitude, etc.) computed by external controllers. Note that for the methods not provided by this package, MAVROS is still the preferred way of communication method for getting information from the UAV and for sending commands.

## Installation

First of all, make sure you have installed the autopilot compatible with this package from [https://github.com/castacks/PX4-fully-actuated](https://github.com/castacks/PX4-fully-actuated). Don't forget to also add the `PX4PATH` environment variable to your `.bashrc` file per the instructions on that repository.

Also, make sure you have a MAVROS version compatible with our PX4 (if you have followed the instructions in our autopilot installation, you will have it in your catkin workspace already).

Now clone the repos into the same catkin workspace as the `Firmware` directory (here we assume it's in `~/catkin_ws/src`) and checkout the branches shown in the links:

```
https://bitbucket.org/castacks/base_main_class/src/master/
https://bitbucket.org/castacks/core_drone_interface/src/build-fixes/
https://bitbucket.org/castacks/core_px4_interface/src/contact-inspection/
```

Now `catkin build` the workspace (remember to run it in `catkin_ws`). It will build these repos as well as the PX4 repo from the `Firmware` directory. Don't forget to source the `devel/setup.bash` in the open terminals. It's easier if you just close the terminals and open new ones.

## How to use

In a new terminal, execute the following command to run PX4 with ROS interface with our fully-actuated UAV hexarotor:

```
roslaunch px4 posix_sitl.launch vehicle:=hexa_x_tilt sdf:=$PX4PATH/Tools/sitl_gazebo/models/hexa_x_tilt/hexa_x_tilt.sdf world:=$PX4PATH/Tools/sitl_gazebo/worlds/grass.world
```

You should see the tilted hex drone in Gazebo now.

In another terminal run MAVROS using this command:

```
roslaunch mavros px4.launch fcu_url:="udp://:14540@localhost:14580"
```

In another terminal run this PX4 interface using this command:

```
roslaunch core_drone_interface drone_interface_node.launch drone_interface:=PX4TiltedHexInterface
```

You can see the the topics and services of this package using the rosservice and rostopic commands or the `rqt` tool.

There is an example code that you can run to see how it all works:

```
rosrun core_px4_interface px4_offboard_example_node
```

## Related Packages

There is a dummy node that mimics the approach of the drone to the contact point. All it does is going forward a bit, stopping and going back. But, it's a good example to see what commands are available and how they can be used. Please see [https://github.com/keipour/contact_interface](https://github.com/keipour/contact_interface) and refer to the readme file of the repo for more information.

Our ROS position controller which uses this interface to control the fully-actuated robot can be accessed from [https://bitbucket.org/castacks/core_pose_controller/src/contact-inspection](https://bitbucket.org/castacks/core_pose_controller/src/contact-inspection).

## Contact
Azarakhsh Keipour (keipour@gmail.com)
