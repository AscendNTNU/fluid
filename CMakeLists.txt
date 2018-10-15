cmake_minimum_required(VERSION 2.8.3)

project(fluid_fsm)


#add_compile_options(-std=c++14)

## Find catkin macros and libraries
#find_package(catkin REQUIRED COMPONENTS
#    ascend_msgs
#    geometry_msgs
#    roscpp
#)
## Includes headers in include folder
#catkin_package(
#    INCLUDE_DIRS include
#    CATKIN_DEPENDS ascend_msgs geometry_msgs roscpp
#)
## Includes headers in include folder
#include_directories(
#include
#    ${catkin_INCLUDE_DIRS}
#)


#########################################################################################

## Adds a ros node with name node_name
#add_executable(off_node src/off_node.cpp)

## Adds a compilation dependency to node_name
#add_dependencies(off_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Links node_name together with other libraries
#target_link_libraries(off_node
#    ${catkin_LIBRARIES}
#)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(SOURCE_FILES src/main.cpp src/operation/graph.cpp src/operation/operation.cpp src/core/transition.cpp include/operation/edge.h include/operation/graph.h include/core/state.h include/operation/operation.h include/core/transition.h include/states/idle_state.h src/states/idle_state.cpp include/states/move_state.h src/states/move_state.cpp include/states/hold_state.h src/states/hold_state.cpp include/operation/state_graph.h src/operation/state_graph.cpp include/states/take_off_state.h include/states/land_state.h src/states/take_off_state.cpp src/states/land_state.cpp)
add_executable(fluid_fsm ${SOURCE_FILES})