cmake_minimum_required(VERSION 2.8.3)
project(draw)

add_compile_options(-std=c++17)

find_package(Eigen3 QUIET)
find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    geometry_msgs
    image_transport
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    actionlib_msgs
    math
)

find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS
    message_runtime
    actionlib_msgs
    cv_bridge
    geometry_msgs
    image_transport
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
    math
)

add_executable(draw_map_cpp 
    src/draw_map.cpp)

set_target_properties(draw_map_cpp 
    PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(draw_map_cpp 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS})

target_link_libraries(draw_map_cpp
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    math
)

add_executable(write_latex
    src/write_latex.cpp)

set_target_properties(write_latex 
    PROPERTIES OUTPUT_NAME write_latex PREFIX "")

add_dependencies(write_latex
    ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(write_latex
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    math
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

install(TARGETS draw_map_cpp write_latex
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)