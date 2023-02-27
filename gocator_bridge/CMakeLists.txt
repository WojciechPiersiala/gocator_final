cmake_minimum_required(VERSION 3.0.2)
project(gocator_bridge)

find_package(PCL REQUIRED) 
find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions pcl_ros
)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rostime
  sensor_msgs
  std_msgs

  tf2
  tf2_ros
)

set(Go_SDK      ${PROJECT_SOURCE_DIR}/lib/libGoSdk.so)
set(kApi        ${PROJECT_SOURCE_DIR}/lib/libkApi.so)

set(LIBRARIES          ${catkin_LIBRARIES} ${Go_SDK} ${kApi})
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES gocator_bridge
 CATKIN_DEPENDS geometry_msgs roscpp rostime sensor_msgs std_msgs pcl_conversions pcl_ros    tf2 tf2_ros
 DEPENDS 
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(PYTHON_SCRIPTS  scripts/view.py)

catkin_install_python(PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(get_profile src/get_profile.cpp)
add_executable(show_sensor_boundries src/show_sensor_boundries.cpp)
add_executable(get_surface_robot src/get_surface_robot.cpp)
add_executable(get_surface_filtered src/get_surface_filtered.cpp)
add_executable(reconstruct_pcl src/reconstruct_pcl.cpp)


target_link_libraries(get_profile ${LIBRARIES})
target_link_libraries(show_sensor_boundries ${catkin_LIBRARIES})
target_link_libraries(get_surface_robot ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(get_surface_filtered ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(reconstruct_pcl ${catkin_LIBRARIES} ${PCL_LIBRARIES})