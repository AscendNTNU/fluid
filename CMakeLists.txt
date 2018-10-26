cmake_minimum_required(VERSION 2.8.3)
project(fluid_fsm)
add_compile_options(-std=c++14)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        ascend_msgs
        geometry_msgs
        roscpp
        genmsg
        actionlib
        )

add_action_files(DIRECTORY action FILES Move.action Land.action TakeOff.action)
generate_messages(DEPENDENCIES geometry_msgs actionlib_msgs)

# Includes headers in include folder
catkin_package(
        CATKIN_DEPENDS ascend_msgs roscpp
)
# Includes headers in include folder
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

file(GLOB fluid_fsm_operation_SRC "src/core/operation/*.cpp")
file(GLOB fluid_fsm_operations_SRC "src/operations/*.cpp")
file(GLOB fluid_fsm_core_SRC "src/core/*.cpp")
file(GLOB fluid_fsm_states_SRC "src/states/*.cpp")
file(GLOB fluid_fsm_actionlib_SRC "src/actionlib/*.cpp")

add_library(fluid_fsm_lib ${fluid_fsm_operation_SRC} ${fluid_fsm_operations_SRC} ${fluid_fsm_core_SRC} ${fluid_fsm_states_SRC} ${fluid_fsm_actionlib_SRC})

target_link_libraries(fluid_fsm_lib ${catkin_LIBRARIES})


#########################################################################################


#set(SOURCE_FILES src/main.cpp

#        src/core/operation/graph.cpp src/core/operation/operation.cpp src/core/operation/state_graph.cpp
#        src/core/transition.cpp
#        src/states/hold_state.cpp src/states/idle_state.cpp src/states/init_state.cpp src/states/move_state.cpp src/states/land_state.cpp src/states/take_off_state.cpp src/states/state_identifier.cpp

#        include/core/operation/graph.h include/core/operation/operation.h include/core/operation/edge.h include/core/operation/state_graph.h
#        include/core/transition.h include/core/state.h
#        include/states/state_identifier.h include/states/idle_state.h include/states/take_off_state.h include/states/land_state.h include/states/move_state.h include/states/hold_state.h include/states/init_state.h include/actionlib/action_server.h include/operations/operation_identifier.h)

add_executable(fluid_fsm_main src/main.cpp)
add_executable(action_interface_client src/off_node.cpp)

target_link_libraries(fluid_fsm_main fluid_fsm_lib)
target_link_libraries(action_interface_client fluid_fsm_lib ${catkin_LIBRARIES})

## Adds a compilation dependency to node_name
add_dependencies(fluid_fsm_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(fluid_fsm_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(action_interface_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})