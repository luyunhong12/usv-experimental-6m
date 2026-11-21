cmake_minimum_required(VERSION 3.16)
project(usv_bringup)
find_package(ament_cmake REQUIRED)
# Development launchers resolve the repository and invoke installed ROS packages.
# They intentionally remain source-tree tools, not misleading installed wrappers.
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
