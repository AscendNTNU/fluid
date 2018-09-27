cmake_minimum_required(VERSION 2.8.3)

project(fluid_fsm)


add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    ascend_msgs
    geometry_msgs
    roscpp
)
## Includes headers in include folder
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ascend_msgs geometry_msgs roscpp
)
## Includes headers in include folder
include_directories(
include
    ${catkin_INCLUDE_DIRS}
)


#########################################################################################

## Adds a ros node with name node_name
add_executable(off_node src/off_node.cpp)

## Adds a compilation dependency to node_name
add_dependencies(off_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Links node_name together with other libraries
target_link_libraries(off_node
    ${catkin_LIBRARIES}
)


