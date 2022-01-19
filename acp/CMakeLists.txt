cmake_minimum_required(VERSION 3.5)
project(sev_acp)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

install(DIRECTORY specs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILE_PERMISSIONS WORLD_READ)

cs_install()
cs_export()
