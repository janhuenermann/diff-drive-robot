cmake_minimum_required(VERSION 2.8.3)
project(local_planner)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
  math
)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES local_planner
   CATKIN_DEPENDS roscpp rospy std_msgs
   # DEPENDS math
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/local_planner/trajectory.cpp
   src/local_planner/pid_controller.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(controller_node src/controller_node.cpp)

set_target_properties(controller_node PROPERTIES OUTPUT_NAME controller PREFIX "")

target_link_libraries(controller_node
   ${catkin_LIBRARIES}
   ${PROJECT_NAME}
   math
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

