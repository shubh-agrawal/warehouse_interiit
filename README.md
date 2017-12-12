## Running the simulator

This setup presumes pre-installed ROS Kinetic with mavros and Gazebo 7 on Ubuntu 16.04

- Get the PX4 autopilot software
``` 
$ mkdir ~/src
$ cd ~/src/
$ git clone https://github.com/PX4/Firmware.git
$ cd Firmware/
$ git submodule update --init --recursive
$ make posix_sitl_default gazebo
```
- Make sure you have all the required [gazebo models](http://machineawakening.blogspot.in/2015/05/how-to-download-all-gazebo-models.html) in ```.gazebo/models/``` folder
- Source the Gazebo Model path and PX4 flight stack
```
$ source ~/src/Firmware/Tools/setup_gazebo.bash ~/src/Firmware ~/src/Firmware/build/posix_sitl_default
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/src/Firmware
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/src/Firmware/Tools/sitl_gazebo
$ export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/warehouse_interiit/models
```
### Dependencies

- Zbar ( for barcoe node ) 

```$ sudo apt-get install ros-<distro>-zbar-ros```
- [WiringPi](http://wiringpi.com/download-and-install/) ( For Sonar on RPi )

- GeographicLib

```$ sudo apt-get install libgeographic-dev```

Once all dependencies are installed 

- Clone and build this repo in your catkin workspace

NOTE : If the build failed to find the Line.h file , comment out the executables and target libraries from the CMakeLists.txt file and build the messegaes first then uncomment the executables and build again.

- Run the simulation
```bash
$ roslaunch warehouse_interiit px4_interiit.launch
$ roslaunch mavros px4.launch fcu_url:="udp://:14540@192.168.1.36:14557"
```
- Once the Simulator boots up run the core nodes

```$ roslaunch warehouse_interiit warehouse.launch```

## Running on Quad

- ```Config.h``` contains all the constants and camera feed strings. Change the values according to the setup.
- Setup the mavlink between the RPi ( or any onboard processor ) 
- Run the strategy, commander, line_detection and barcode node.

For Running OnBoard 
```
$ roslaunch warehouse_interiit warehouse.launch
```

For Running OffBoard ,create a ROS_MASTER and create a multi PC ROS

```
$ roslaunch warehouse_interiit warehouseOff.launch	#Onboard 
$ rosrun warehouse_interiit line_detect				#On PC
```

Published Topics :
```
/altitude_setpoint     		#Sends Target Altitude
/cgo3_camera/image_raw		#Downward Facing Camera Video Feed
/code/bar					#Barcode decoded Strings 
/code/image					#Detected Bar/QR Code image with bounding box
/code/qr					#QR Coded decoded string
/code/scan					#Boolean to signal the trigger of scanning	
/feedback/horizontal		#Current Target / Hoverig Line
/feedback/vertical			#Current Vertical Following Line
/lines/horizontal			#Array of Horizontal Detetcted Lines
/lines/vertical				#Vertical Detetcted Line 
/right_camera/image_raw		#Barcode Scanning Side Facing Cam Feed
/state						#String to tell controller to switch states
/edge				#Boolean to publish vertical edge line array in alternate algorithm
/square/vertical		#Publishes vertical line from centre of the square landing area
/square/horizontal		#Publishes horizontal line from centre of the square landing area
```

#### Custom Messages

```Line```  Stores ```rho``` and ```theta``` of the lines.

```Linearray``` Stores the array of Lines.

### Algorithm Overview

Color segmentation based on HSV color space is done on the down ward facing camera to locate the lines for quad to follow. Whole image is divided into ```N_SLICE_H``` x ```N_SLICE_W``` ( declared in Config.h ) parts and contour detection is applied on each one of the segment to form multiple circles outlining the grid lines. ```Hough Line``` is used on the circles to find the lines passing through the circles in a binary image.

Clustering Algorithm is used to find the best fitting line and publishes the line in vertical and array of lines in horizontal direction.


