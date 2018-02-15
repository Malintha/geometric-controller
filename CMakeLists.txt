cmake_minimum_required(VERSION 2.8.3)
project(geo_controller)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    tf
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(geo_controller
        src/GeoController.cpp
        src/controllerImpl.h
        src/dynamicsImpl.h
        src/geoControlllerUtils.h)

target_link_libraries(geo_controller
        ${catkin_LIBRARIES})