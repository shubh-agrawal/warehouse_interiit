cmake_minimum_required(VERSION 2.8.3)
project(warehouse_interiit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_geometry
  geometry_msgs
  mavros_msgs
  sensor_msgs
  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED)
find_package (ZBar0)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS cv_bridge roscpp 
)
add_executable(barcode_node
	src/barcode.cpp
  )
target_link_libraries(barcode_node zbar ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_executable(takeoff_node src/takeoff.cpp)
target_link_libraries(takeoff_node
   ${catkin_LIBRARIES}
)

