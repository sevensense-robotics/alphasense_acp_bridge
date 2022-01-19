cmake_minimum_required(VERSION 3.5)
project(sev_acp_ros_conversions)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} src/conversions.cpp)

cs_install()
cs_export()
