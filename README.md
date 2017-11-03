## Running the simulator

This setup presumes pre-installed ROS Kinetic with mavros and GAzebo 7 on Ubuntu 16.04

- Get the PX4 autopilot software
```bash
mkdir ~/src
cd ~/src/
git clone https://github.com/PX4/Firmware.git
cd Firmware/
git submodule update --init --recursive
make posix_sitl_default gazebo
```
- Make sure you have all the required [gazebo models](http://machineawakening.blogspot.in/2015/05/how-to-download-all-gazebo-models.html) in ```.gazebo/models/``` folder
- Source the PX4 flight stack
```bash
source ~/src/Firmware/Tools/setup_gazebo.bash ~/src/Firmware ~/src/Firmware/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/src/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/src/Firmware/Tools/sitl_gazebo
```
- Clone and build this repo in your catkin workspace
- Run the simulation
```bash
roslaunch warehouse_interiit px4_interiit.launch
```
