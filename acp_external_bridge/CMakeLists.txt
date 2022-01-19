cmake_minimum_required(VERSION 3.5)
project(sev_acp_external_bridge)

find_package(catkin_simple REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

catkin_simple()

cs_add_executable(ap-udp-bridge src/bridge.cc src/cli.cpp)
target_link_libraries(ap-udp-bridge ${Boost_LIBRARIES})

cs_install()
cs_export()
