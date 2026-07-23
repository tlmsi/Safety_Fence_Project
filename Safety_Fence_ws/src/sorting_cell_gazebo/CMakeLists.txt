cmake_minimum_required(VERSION 3.8)

project(sorting_cell_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)

install(
  DIRECTORY
    launch
    models
    worlds
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
