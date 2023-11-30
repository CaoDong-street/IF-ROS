cmake_minimum_required(VERSION 2.8.3)
project(multi_main_controller)

find_package(
    catkin REQUIRED
    COMPONENTS roscpp
               std_msgs
               geometry_msgs
               nav_msgs
               mavros_msgs
               tf)

find_package(Eigen3 REQUIRED)

set(THIRD_PARTY_LIBS pthread ${catkin_LIBRARIES})

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
catkin_package()

add_library(loadController include/loadController/flightController.h src/loadController/flightController.cpp)
target_link_libraries(loadController ${catkin_LIBRARIES} ${THIRD_PARTY_LIBS})

add_library(trajectoryGenerator include/trajGenerator/trajectory_generator_waypoint.h
                                src/trajGenerator/trajectory_generator_waypoint.cpp)
target_link_libraries(trajectoryGenerator ${catkin_LIBRARIES})

add_executable(
    takeoff
    App/takeoff.cpp
    src/loadController/math.cpp
    include/loadController/common_include.h
    include/loadController/math.h)
target_link_libraries(
    takeoff
    ${THIRD_PARTY_LIBS}
    loadController
    trajectoryGenerator)
