cmake_minimum_required(VERSION 3.14)
project(tvp_launch)

find_package(autoware_cmake REQUIRED)
autoware_package()

autoware_ament_auto_package(
  INSTALL_TO_SHARE
  launch
  rviz
  config
)
