cmake_minimum_required(VERSION 2.8.3)
project(meshing_plugins_base)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    pcl_ros
    roscpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


#############
## Install ##
#############

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
