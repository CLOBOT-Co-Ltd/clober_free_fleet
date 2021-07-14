cmake_minimum_required(VERSION 3.0.2)

project(clober_ff_client_ros1)

#===============================================================================

find_package(catkin QUIET)

if (catkin_FOUND)

  catkin_package()

  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )

else()
  message("catkin not found so skipping this ROS1 package")
endif()
