cmake_minimum_required(VERSION 3.5)
project(sev_acp_udp)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(7s-acp-udp-echo src/inspector.cpp NO_AUTO_LINK)

cs_install()
cs_export()
