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
        tf2
        )

add_action_files(DIRECTORY action FILES Operation.action)
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
file(GLOB fluid_fsm_navigator_SRC "src/navigator/*.cpp")
file(GLOB fluid_fsm_tools_SRC "src/tools/*.cpp")

add_library(fluid_fsm_lib
        ${fluid_fsm_mavros_SRC}
        ${fluid_fsm_navigator_SRC}
        ${fluid_fsm_operation_SRC}
        ${fluid_fsm_operations_SRC}
        ${fluid_fsm_core_SRC}
        ${fluid_fsm_states_SRC}
        ${fluid_fsm_actionlib_SRC}
        ${fluid_fsm_tools_SRC})

target_link_libraries(fluid_fsm_lib ${catkin_LIBRARIES})


#########################################################################################

add_executable(main src/main.cpp)
add_executable(test_client src/test_client.cpp)
add_executable(test_interval_client src/test_interval_client.cpp)
add_executable(graph_test src/graph_test.cpp)

target_link_libraries(main fluid_fsm_lib)
target_link_libraries(test_client fluid_fsm_lib ${catkin_LIBRARIES})
target_link_libraries(test_interval_client fluid_fsm_lib ${catkin_LIBRARIES})
target_link_libraries(graph_test fluid_fsm_lib ${catkin_LIBRARIES})

## Adds a compilation dependency to node_name
add_dependencies(fluid_fsm_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(graph_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
