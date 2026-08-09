cmake_minimum_required(VERSION 3.22.1)

project(sorting_cell_gazebo)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ros_gz_sim REQUIRED)

find_package(
  Qt6 6.4
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED
)

find_package(gz-gui REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz-msgs REQUIRED)


# ------------------------------------------------------------
# Gazebo GUI safety control panel
# ------------------------------------------------------------

qt_add_resources(
  SAFETY_PANEL_RCC
  gui/SafetyPanel.qrc
)

add_library(
  SafetyPanel
  SHARED
  gui/SafetyPanel.hh
  gui/SafetyPanel.cc
  ${SAFETY_PANEL_RCC}
)

target_link_libraries(
  SafetyPanel
  PRIVATE
    Qt6::Core
    Qt6::Quick
    Qt6::QuickControls2
    gz-gui::gz-gui
    gz-transport::gz-transport
    gz-msgs::gz-msgs
)


# ------------------------------------------------------------
# Gazebo world safety runtime
# ------------------------------------------------------------

add_library(
  SafetyRuntime
  SHARED
  plugins/SafetyRuntime.cc
)

target_link_libraries(
  SafetyRuntime
  PRIVATE
    gz-sim::gz-sim
    gz-transport::gz-transport
    gz-msgs::gz-msgs
)


install(
  TARGETS
    SafetyPanel
    SafetyRuntime
  LIBRARY DESTINATION lib
)


install(
  DIRECTORY
    launch
    models
    worlds
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
