cmake_minimum_required(VERSION 2.8.3)
project(fluid)
add_compile_options(-std=c++14)

#########################################################################################

find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        roscpp
        ascend_msgs
        std_msgs
        visualization_msgs
        tf2_geometry_msgs
        tf2
        message_generation
        dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/Server.cfg
)

add_service_files(
        FILES
        Explore.srv
        ExtractModule.srv
        Land.srv
        TakeOff.srv
        Travel.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS message_runtime ascend_msgs roscpp dynamic_reconfigure 
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
add_executable(publisher          src/nodes/publisher_node.cpp                  ${fluid_SRC} ${fluid_states_SRC})
add_executable(test_client        src/test/test_client.cpp                      ${fluid_SRC} ${fluid_states_SRC})

add_dependencies(server                  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_dependencies(publisher               ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_client             ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(server             ${catkin_LIBRARIES})
target_link_libraries(publisher          ${catkin_LIBRARIES})
target_link_libraries(test_client        ${catkin_LIBRARIES})