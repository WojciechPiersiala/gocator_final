cmake_minimum_required(VERSION 3.0.2)
project(gocator_sim)


find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_planning_interface
  std_msgs
  moveit_visual_tools
  rviz_visual_tools
  tf2
)
# generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gocator_sim
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

set(PYTHON_SCRIPTS  scripts/ros_gui.py)

catkin_install_python(PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(move_arm src/move_arm.cpp)
add_executable(move_arm_init src/move_arm_init.cpp)
add_executable(send_command src/send_command.cpp)
add_executable(move_trajectory src/move_trajectory.cpp)

target_link_libraries(move_arm ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(move_arm_init ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(send_command ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(move_trajectory ${catkin_LIBRARIES} ${Boost_LIBRARIES})

