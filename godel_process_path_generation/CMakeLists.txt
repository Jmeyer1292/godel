cmake_minimum_required(VERSION 2.8.3)
project(godel_process_path_generation)
add_definitions(-std=c++11 )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  message_generation
  godel_msgs
  visualization_msgs
  geometry_msgs
  trajectory_msgs
  tf
  eigen_conversions
  moveit_core
  moveit_ros_planning_interface
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES VisualizeBlendingPlan.srv
)

# Trajectory msgs needed for building msg in Visualize...srv
generate_messages(DEPENDENCIES 
  geometry_msgs
  godel_msgs
  visualization_msgs
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}_polygon_utils
  CATKIN_DEPENDS
    roscpp
    pcl_ros
    message_generation
    godel_msgs
    visualization_msgs
    geometry_msgs
    trajectory_msgs
    tf
    eigen_conversions
    moveit_core
    moveit_ros_planning_interface
)

###########
## Build ##
###########

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS}
)

##_____________
## Libraries ##



## ProcessPath library
add_library(process_path
            src/process_path.cpp
            src/process_pt.cpp
            src/process_transition.cpp
)
target_link_libraries(process_path
                      ${catkin_LIBRARIES}
                      ${Eigen_LIBRARIES}
)

## Polygon Utilities library
add_library(${PROJECT_NAME}_polygon_utils
            src/polygon_utils.cpp
)

target_link_libraries(${PROJECT_NAME}_polygon_utils
                      ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_polygon_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## ProcessPathGenerator library
add_library(process_path_generator
            src/process_path_generator.cpp
)
target_link_libraries(process_path_generator
                      process_path
                      ${PROJECT_NAME}_polygon_utils
)

##_________
## Nodes ##
## Process Path Generator node
add_executable(process_path_generator_node
               src/process_path_generator_node.cpp
)
target_link_libraries(process_path_generator_node
                      process_path
                      process_path_generator
)
add_dependencies(process_path_generator_node godel_msgs_generate_messages_cpp)


#############
## Testing ##
#############

##__________________
## Visual Testing ##
## Process Planner Visualization node
add_executable(process_visualization_node
               src/process_path_visualization_node
)
target_link_libraries(process_visualization_node
                      ${catkin_LIBRARIES}
)
add_dependencies(process_visualization_node ${PROJECT_NAME}_generate_messages_cpp)


## gtest ##
catkin_add_gtest(test_ProcessPathGenerator test/test_ProcessPathGenerator.cpp)
target_link_libraries(test_ProcessPathGenerator
                      process_path_generator
)

catkin_add_gtest(test_PolygonUtils test/test_polygon_utils.cpp)
target_link_libraries(test_PolygonUtils
                      ${PROJECT_NAME}_polygon_utils
)
