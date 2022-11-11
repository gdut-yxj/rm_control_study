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
add_library(left_front_wheel_controller
        src/left_front_wheel_controller.cpp)

## Specify libraries to link executable targets against
target_link_libraries(left_front_wheel_controller
        ${catkin_LIBRARIES})

#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)