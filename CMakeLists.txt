cmake_minimum_required(VERSION 2.8.3)
project(scan_fining)

set(CMAKE_CXX_FLAGS "-std=c++0x -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf
)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scan_fining
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf
    message_runtime
  DEPENDS
    system_lib
)

add_executable(scan_fining src/scan_fining.cpp)
add_dependencies(scan_fining
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(scan_fining ${catkin_LIBRARIES})
