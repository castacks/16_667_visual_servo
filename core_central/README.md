# Core Central for Aerial Manipulation


### Install Dependencies

Deactivate your conda environment if any of that is on, e.g. (base) to avoid messing up with the python packages. In addition, because Ubuntu no longer has ```python``` command, ```python3``` instead, to use the old software stack from 18.04, let's do a python-python3 mapping: 

```
sudo apt install python-is-python3
echo "alias python='python3'"  >> ~/.bashrc
source ~/.bashrc
```

Follow the guidance of the [ROS installation tutorial for Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) to install ROS Noetic properly. In particular, the **1.5 Environment Setup** and **1.6 Dependencies for Building Packages** are very important.

Then, run the followings to install other dependencies:
```
sudo apt install -y emacs python3-jinja2 python3-wstool python3-pip libgeographic-dev geographiclib-tools  python3-catkin-tools libusb-dev libsuitesparse-dev ros-noetic-geographic-msgs ros-noetic-serial ros-noetic-rosmon* ros-noetic-jsk-rviz-plugins ros-noetic-joy libglfw3-dev libblosc-dev libopenexr-dev liblog4cplus-dev libpcap-dev opencl-headers
pip install wheel numpy toml future serial
```

As for PX4 related dependencies, follow the guidance of [PX4 documentation](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html) to install the toolchain. In particular, although the [ubuntu_sim_ros_melodic.sh](https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh) is for melodic, the common tool inside is still worth to install.

### Building the Code

The general pattern for building the code is to pull this repo, create a symbolic link to the rosinstall for one of the platforms, pull the repos with `wstool update`, then build with `catkin build`. There may be additional platform specific steps as well.

### Launch File Structure

The launch files are organized into directories for each platform, gazebo/, px4/, and dji/, along with a common/ with launch files usable by each platform. The simulation example launch files have the following format `[platform name]_example_[example type].launch`. The `platform name` will be either gazebo, px4, or dji. The example type describes the example being run, for example a lidar based drone, disparity based drone, or multiple drone simulation. Inside the example launch files, gazebo is launched with a world file and one or more simulated drones running the software stack are created by including a `[platform name]_sim_drone.launch` launch file one or more times.

The `[platform name]_sim_drone.launch` launch files include launch files to spawn the drone in sim, run state estimation, run control, run the rest of the autonomy in the core stack, and run some visualization including rviz and rqt. The state estimation and control are placed in separate launch files because usually the state estimation and control gains are different between sim and real life, so for a real drone you would swap these two launch files out. The launch file that runs the rest of the autonomy can remain the same and should run onboard the drone. The visualization can also remain the same but should run on the base station computer you are using to communicate with the drone.

### Using the stack in your project

When modifying a package in the stack for your project specific needs, you should create a branch with your project's name in that package. The branch that comes with the core autonomy stack (usually master) is locked so that only people who are maintainers of the stack can push to it. If you have a feature you would like to be merged into the core you should create a pull request.

### Bugs / Feedback

If you encounter any bugs or have feedback about how the software or documentation could be improved, contact John Keller (slack: kellerj).


## PX4

### Building the Code

The main difference between the pure gazebo build and the px4 build, is that you must build the firmware, using `make px4_sitl_default gazebo` in addition to doing `catkin build`. Running `make px4_sitl_default gazebo` builds the firmware AND launchs a gazebo simulation. If you just do `make px4_sitl_default`, the px4 sitl will not respond to mavros. With the `gazebo` line it does work but launches gazebo when the build is finished, so you need to manually Ctrl-C on the terminal when gazebo pops up so you can continue building the code. Do the following:


### Install dependencies for Apriltag_ros
Use Rosdep to install dependencies residual dependencies for Apriltag pakages
```
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
```
For installing missing dependencies for packages:
```
rosdep install --from-paths src --ignore-src -r -y
```
### Intall xmlStartlet 
```
sudo apt-get install xmlstarlet
```

### Build Sim Workspace
```
mkdir -p ws/src
cd ws/src
```
Extract the zip file on teamforge here in ws/src

```
cd Firmware
make px4_sitl_default gazebo # This will build the firmware and launch a gazebo simulation. When the gazebo window shows up, Ctrl-C on the command line and run the next commands.
```
Make sure you ahve installed Ros noetic and mavros datasets
```
ros-noetic-desktop-full
the following steps only need to be done once to install the datasets:
cd ../mavros/mavros/scripts
sudo su
./install_geographiclib_datasets.sh
```
then you can build the workspace as usual:
```
cd /path/to/workspace/
catkin build
```

### Running the Examples

The examples should look the same as the pure gazebo version, but with a px4 drone. Run the commands listed below.

#### Example
Run Master in separate terminal
```
roscore
```

```
mon launch core_central champ_control.launch
```




