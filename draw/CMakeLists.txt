cmake_minimum_required(VERSION 2.8.3)
project(draw)
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    geometry_msgs
    image_transport
    message_generation
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    actionlib_msgs
)
find_package(OpenCV REQUIRED)
generate_messages(DEPENDENCIES
    actionlib_msgs
    std_msgs
)
catkin_package(CATKIN_DEPENDS
    message_runtime
    actionlib_msgs
    cv_bridge
    geometry_msgs
    image_transport
    message_generation
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
)

add_executable(draw_map_cpp src/draw_map.cpp)
set_target_properties(draw_map_cpp PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(draw_map_cpp ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(draw_map_cpp
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

install(TARGETS draw_map_cpp
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)