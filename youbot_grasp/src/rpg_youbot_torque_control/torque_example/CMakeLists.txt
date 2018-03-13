cmake_minimum_required(VERSION 2.8.3)
project(torque_example)

find_package(catkin REQUIRED COMPONENTS
  brics_actuator
  tf_conversions
  roscpp
  ik_solver_service
  brics_actuator
  trajectory_msgs
  rpg_youbot_common
  torque_control
  trajectory_generator
  message_generation
  cmake_modules
  moveit_core
  pluginlib
  actionlib
  roscpp
  rosconsole
  urdf
  kdl_parser
  tf
  tf_conversions
  eigen_conversions
  control_msgs
             moveit_ros_planning
             moveit_ros_planning_interface
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

find_package(Boost REQUIRED thread)

catkin_package(
  CATKIN_DEPENDS
    torque_control
    brics_actuator
    trajectory_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  
)
add_executable(base src/base.cpp)
add_dependencies(base ${catkin_EXPORTED_TARGETS})

target_link_libraries(base
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_executable(ltr src/ltr.cpp)
add_dependencies(ltr ${catkin_EXPORTED_TARGETS})

target_link_libraries(ltr
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(ltr1 src/ltr1.cpp)
add_dependencies(ltr1 ${catkin_EXPORTED_TARGETS})

target_link_libraries(ltr1
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(ltr2 src/ltr2.cpp)
add_dependencies(ltr2 ${catkin_EXPORTED_TARGETS})

target_link_libraries(ltr2
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(ltr3 src/ltr3.cpp)
add_dependencies(ltr3 ${catkin_EXPORTED_TARGETS})

target_link_libraries(ltr3
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)


add_executable(linux_new_ltr3 src/linux_new_ltr3.cpp)
add_dependencies(linux_new_ltr3 ${catkin_EXPORTED_TARGETS})

target_link_libraries(linux_new_ltr3
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)



add_executable(testfk src/testfk.cpp)
add_dependencies(testfk ${catkin_EXPORTED_TARGETS})

target_link_libraries(testfk
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(circle_traj src/circle.cpp)
add_dependencies(circle_traj ${catkin_EXPORTED_TARGETS})

target_link_libraries(circle_traj
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)



