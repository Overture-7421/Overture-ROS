cmake_minimum_required(VERSION 3.0.2)
project(overture_gazebo_plugins)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++20)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
        roscpp
        gazebo_ros
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS} /usr/local/lib)
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} /usr/local/include/wpimath /usr/local/include/ntcore /usr/local/include/wpiutil)

catkin_package(
        DEPENDS
        roscpp
        gazebo_ros
)

add_library(nt_motor_plugin SHARED src/nt_motor.cpp)
target_link_libraries(nt_motor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} wpiutil wpimath ntcore)