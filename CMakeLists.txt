cmake_minimum_required(VERSION 3.10)
project(rm_control_study)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ROS_DEPENDENCIES
        roscpp
        rm_common

        controller_interface
        effort_controllers
        )

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES})

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        CATKIN_DEPENDS
        ${ROS_DEPENDENCIES}
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


## Declare a cpp library
add_library(joint1_vel_controller
        src/joint1_vel_controller.cpp)

## Specify libraries to link executable targets against
target_link_libraries(joint1_vel_controller
        ${catkin_LIBRARIES})
