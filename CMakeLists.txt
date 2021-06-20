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
        tf2
        tf2_geometry_msgs
        tf2_ros
        message_generation
)

add_service_files(
        FILES
        OperationCompletion.srv
        Explore.srv
        Interact.srv
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
        CATKIN_DEPENDS message_runtime ascend_msgs roscpp 
)

#########################################################################################

include_directories(include
        include/fluid
        include/fluid/operations
        ${catkin_INCLUDE_DIRS}
)

file(GLOB fluid_SRC "src/*.cpp")
file(GLOB fluid_operations_SRC "src/operations/*.cpp")

#########################################################################################

add_executable(fluid              src/nodes/main.cpp                            ${fluid_SRC} ${fluid_operations_SRC})
add_executable(example_client     src/examples/example_client.cpp               ${fluid_SRC} ${fluid_operations_SRC})
add_executable(follow_reference     src/examples/follow_reference.cpp               ${fluid_SRC} ${fluid_operations_SRC})
add_executable(base_link_publisher     src/nodes/base_link_publisher.cpp)

add_dependencies(fluid                   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(example_client          ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(follow_reference          ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(base_link_publisher          ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(fluid              ${catkin_LIBRARIES})
target_link_libraries(example_client     ${catkin_LIBRARIES})
target_link_libraries(follow_reference     ${catkin_LIBRARIES})
target_link_libraries(base_link_publisher     ${catkin_LIBRARIES})
