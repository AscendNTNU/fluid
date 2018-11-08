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

add_action_files(DIRECTORY action FILES Move.action Land.action TakeOff.action Init.action)
generate_messages(DEPENDENCIES geometry_msgs actionlib_msgs)

catkin_package(
        CATKIN_DEPENDS ascend_msgs roscpp
)

# Includes headers in include folder
include_directories(
        include
        tests/include
        ${catkin_INCLUDE_DIRS}
)

file(GLOB fluid_fsm_operation_SRC "src/core/operation/*.cpp")
file(GLOB fluid_fsm_operations_SRC "src/operations/*.cpp")
file(GLOB fluid_fsm_core_SRC "src/core/*.cpp")
file(GLOB fluid_fsm_states_SRC "src/states/*.cpp")
file(GLOB fluid_fsm_actionlib_SRC "src/actionlib/*.cpp")
file(GLOB fluid_fsm_mavros_SRC "src/mavros/*.cpp")
file(GLOB fluid_fsm_tests_SRC "tests/src/*.cpp")

add_library(fluid_fsm_lib
        ${fluid_fsm_mavros_SRC}
        ${fluid_fsm_operation_SRC}
        ${fluid_fsm_operations_SRC}
        ${fluid_fsm_core_SRC}
        ${fluid_fsm_states_SRC}
        ${fluid_fsm_actionlib_SRC}
        ${fluid_fsm_tests_SRC})

target_link_libraries(fluid_fsm_lib ${catkin_LIBRARIES})


#########################################################################################

add_executable(fluid_fsm_main src/main.cpp)
add_executable(action_interface_client src/off_node.cpp)
add_executable(transition_test tests/src/transition_test.cpp)

target_link_libraries(fluid_fsm_main fluid_fsm_lib)
target_link_libraries(action_interface_client fluid_fsm_lib ${catkin_LIBRARIES})
target_link_libraries(transition_test fluid_fsm_lib)

## Adds a compilation dependency to node_name
add_dependencies(fluid_fsm_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(fluid_fsm_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(action_interface_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(transition_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})