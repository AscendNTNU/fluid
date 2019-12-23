cmake_minimum_required(VERSION 2.8.3)
project(fluid)
add_compile_options(-std=c++14)

#########################################################################################

find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        ascend_msgs
        roscpp
        genmsg
        actionlib
        visualization_msgs
        tf2_geometry_msgs
        tf2
        dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/Server.cfg
)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS ascend_msgs roscpp dynamic_reconfigure 
)

#########################################################################################

include_directories(include
        include/fluid
        include/fluid/states
        ${catkin_INCLUDE_DIRS}
)

file(GLOB fluid_SRC "src/*.cpp")
file(GLOB fluid_states_SRC "src/states/*.cpp")

#########################################################################################

add_executable(server             src/nodes/main.cpp                            ${fluid_SRC} ${fluid_states_SRC})
add_executable(setpoint_publisher src/nodes/setpoint_publisher_node.cpp         ${fluid_SRC} ${fluid_states_SRC})
add_executable(test_client        src/test/test_client.cpp                      ${fluid_SRC} ${fluid_states_SRC})

add_dependencies(server                  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(setpoint_publisher      ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_client             ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(server             ${catkin_LIBRARIES})
target_link_libraries(setpoint_publisher ${catkin_LIBRARIES})
target_link_libraries(test_client        ${catkin_LIBRARIES})