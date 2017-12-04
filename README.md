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
roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
```

## Running Barcode Reader

Implements zbar for ROS. Subscribes to ```/usb_cam/image_raw``` and publishes detected results under `/code`.

- ```/code/image``` displays final image after processing

- ```/code/qr``` contains the qr code data if one is encountered.

- ```/code/bar``` contains any other barcode data if one in encountered.

Run ```usb_cam``` node before running ```barcode_node```

Requirements :
OpenCV

Zbar   ```sudo apt-get install ros-<distro>-zbar-ros```
 
usb_cam    ```sudo apt-get install ros-<distro>-usb-cam ```
