cmake_minimum_required(VERSION 2.8.3)
project(fluid)
add_compile_options(-std=c++14)

# Find catkin macros and libraries

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen3)

find_package(catkin REQUIRED COMPONENTS
        ascend_msgs
        roscpp
        genmsg
        actionlib
        tf2
)
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS ascend_msgs roscpp
        LIBRARIES ${PROJECT_NAME}
)

# Includes headers in include folder
include_directories(include
        include/fluid
        include/fluid/states
        include/fluid/controllers
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
)

# Retrieve all the source files using GLOB
file(GLOB fluid_SRC "src/*.cpp")
file(GLOB fluid_states_SRC "src/states/*.cpp")
file(GLOB fluid_controllers_SRC "src/controllers/*.cpp")

# Add the source files to the library
add_library(fluid
            ${fluid_SRC}
            ${fluid_controllers_SRC}
            ${fluid_states_SRC})

target_link_libraries(fluid ${catkin_LIBRARIES})

#########################################################################################

add_executable(server src/nodes/main.cpp)
add_executable(setpoint_publisher src/nodes/setpoint_publisher_node.cpp)
add_executable(test_client src/test/test_client.cpp)

target_link_libraries(server fluid              ${catkin_LIBRARIES})
target_link_libraries(setpoint_publisher fluid  ${catkin_LIBRARIES})
target_link_libraries(test_client fluid         ${catkin_LIBRARIES})

## Adds a compilation dependency to node_name
add_dependencies(server                 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(setpoint_publisher     ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_client            ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

