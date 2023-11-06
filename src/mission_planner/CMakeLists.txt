cmake_minimum_required(VERSION 3.8)
project(mission_planner)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces)
find_package(btcpp_ros2_interfaces REQUIRED)

include_directories(
  include
)

set(THIS_PACKAGE_DEPS 
    behaviortree_ros2
    std_msgs
    custom_interfaces
    btcpp_ros2_interfaces)

######################################################
# Build a client that call the sleep action (STATIC version)

add_executable(mission_planner 
  src/control_action.cpp
  src/sleep_action.cpp
  src/mission_planner.cpp)
ament_target_dependencies(mission_planner ${THIS_PACKAGE_DEPS})

######################################################
# Build Server
add_executable(control_server 
src/control_server.cpp)
ament_target_dependencies(control_server ${THIS_PACKAGE_DEPS}) 

add_executable(sleep_server 
src/sleep_server.cpp)
ament_target_dependencies(sleep_server ${THIS_PACKAGE_DEPS}) 

######################################################
# INSTALL

install(TARGETS
  mission_planner
  control_server
  sleep_server
  DESTINATION lib/${PROJECT_NAME}
  )


ament_export_dependencies(behaviortree_ros2 btcpp_ros2_interfaces)

ament_package()
