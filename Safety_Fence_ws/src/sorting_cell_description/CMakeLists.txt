cmake_minimum_required(VERSION 3.8)

project(sorting_cell_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY
    urdf
    launch
    rviz
    meshes
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
