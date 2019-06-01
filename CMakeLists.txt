cmake_minimum_required(VERSION 2.8.3)
project(fluid)
add_compile_options(-std=c++14)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        ascend_msgs
        roscpp
        genmsg
        actionlib
        tf2
)

add_action_files(DIRECTORY action FILES Operation.action)
generate_messages(DEPENDENCIES geometry_msgs actionlib_msgs)

# Includes headers in include folder
include_directories(include
                   
include/fluid/core
include/fluid/operations
include/fluid/states
include/fluid/tools
${catkin_INCLUDE_DIRS}
)

# Retrieve all the source files using GLOB
file(GLOB fluid_fsm_operations_SRC "src/operations/*.cpp")
file(GLOB fluid_fsm_core_SRC "src/core/*.cpp")
file(GLOB fluid_fsm_states_SRC "src/states/*.cpp")

# Add the source files to the library
add_library(fluid
        ${fluid_fsm_operations_SRC}
        ${fluid_fsm_core_SRC}
        ${fluid_fsm_states_SRC})

target_link_libraries(fluid ${catkin_LIBRARIES})

# Export the library
catkin_package(
        CATKIN_DEPENDS ascend_msgs roscpp
        INCLUDE_DIRS include
        LIBRARIES fluid
)

## Mark libraries for installation
install(TARGETS fluid
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

## Mark header files for installation
install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)

#########################################################################################

add_executable(server src/main.cpp)
add_executable(setpoint_publisher src/setpoint_publisher_node.cpp)
add_executable(client_square src/client_square.cpp)
add_executable(client_interval src/client_interval.cpp)
add_executable(client_tracking src/client_tracking.cpp)
add_executable(tracking_publisher src/tracking_pose_publisher.cpp)
add_executable(client_random_movement src/client_random_movement.cpp)

target_link_libraries(server fluid ${catkin_LIBRARIES})
target_link_libraries(setpoint_publisher fluid ${catkin_LIBRARIES})
target_link_libraries(client_square fluid ${catkin_LIBRARIES})
target_link_libraries(client_interval fluid ${catkin_LIBRARIES})
target_link_libraries(client_tracking fluid ${catkin_LIBRARIES})
target_link_libraries(tracking_publisher fluid ${catkin_LIBRARIES})
target_link_libraries(client_random_movement fluid ${catkin_LIBRARIES})

## Adds a compilation dependency to node_name
add_dependencies(server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(setpoint_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(client_square ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(client_interval ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(client_tracking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(tracking_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(client_random_movement ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

