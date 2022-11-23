cmake_minimum_required(VERSION 3.0.2)
project(create_tools)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS
  cv_bridge
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
)