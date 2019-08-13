cmake_minimum_required(VERSION 2.8.3)
project(scan_fining)

set(CMAKE_CXX_FLAGS "-std=c++0x -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs nav_msgs geometry_msgs tf message_generation)

#add_message_files(FILES CircleObstacle.msg SegmentObstacle.msg Obstacles.msg)
add_message_files(FILES)
#generate_messages(DEPENDENCIES std_msgs geometry_msgs)
generate_messages(DEPENDENCIES sensor_msgs geometry_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES scan_fining
   CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs tf message_runtime
   DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(scan_fining src/scan_fining.cpp include/scan_fining.h include/point.h)
target_link_libraries(scan_fining ${catkin_LIBRARIES})
add_dependencies(scan_fining ${${PROJECT_NAME}_EXPORTED_TARGETS})

#add_executable(obstacle_detector src/obstacle_detector.cpp include/obstacle_detector.h
#               include/point.h include/segment.h include/circle.h include/figure_fitting.h)
#target_link_libraries(obstacle_detector ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})
#add_dependencies(obstacle_detector ${${PROJECT_NAME}_EXPORTED_TARGETS})

