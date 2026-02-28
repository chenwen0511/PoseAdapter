cmake_minimum_required(VERSION 3.0.2)
project(pose_adapter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  tf
  dynamic_reconfigure
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    cv_bridge
    image_transport
    tf
    dynamic_reconfigure
)

install(PROGRAMS
  scripts/pose_adapter_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY
  config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)