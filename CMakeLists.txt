cmake_minimum_required(VERSION 3.0.2)
project(robot_traj_generator)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  rosdyn_core
  moveit_core
  moveit_ros_planning_interface
  tf_conversions
  eigen_conversions
  moveit_planning_helper
  gt_coop_mpc
  )

catkin_package(
  INCLUDE_DIRS 
  include
  CATKIN_DEPENDS
  rosdyn_core
)
include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_executable(robot_traj_generator src/robot_traj_generator.cpp)
target_link_libraries( robot_traj_generator ${catkin_LIBRARIES} )
add_dependencies(robot_traj_generator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable(mpc_cart_traj_pub src/mpc_cart_traj_pub.cpp)
target_link_libraries( mpc_cart_traj_pub ${catkin_LIBRARIES} )
add_dependencies(mpc_cart_traj_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})




