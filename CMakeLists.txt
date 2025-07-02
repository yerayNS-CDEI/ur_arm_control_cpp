cmake_minimum_required(VERSION 3.8)
project(ur_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_ros_move_group REQUIRED)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

find_package(ur_robot_driver REQUIRED)

# #--------------------------------------------------------------------
# # set dependencies
# set(dependencies
# rclcpp
# rclpy
# )

#C++ --------------------------------------------------------------------
#include directories - here we can add header files
include_directories(include)

# add executables for scripts
add_executable(simple_node_cpp src/simple_node.cpp)
ament_target_dependencies(simple_node_cpp rclcpp)
# ament_target_dependencies(simple_node_cpp ${dependencies})

add_executable(my_moveit_program src/my_moveit_program.cpp)
ament_target_dependencies(my_moveit_program 
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  moveit_ros_planning
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_servo
  moveit_ros_perception
  moveit_ros_move_group
  std_srvs
)

add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
ament_target_dependencies(servo_keyboard_input 
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  moveit_ros_planning
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_servo
  moveit_ros_perception
  moveit_ros_move_group
)

target_link_libraries(my_moveit_program
  GLUT::GLUT
  OpenGL::GL
)

# install the scripts
install(TARGETS
  simple_node_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  my_moveit_program
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
servo_keyboard_input
  DESTINATION lib/${PROJECT_NAME}
)

#PY --------------------------------------------------------------------
#install python modules
ament_python_install_package(${PROJECT_NAME})

#install python executables
install(PROGRAMS
  scripts/simple_node.py
  scripts/example_move.py
  scripts/publisher_joint_trajectory_controller.py
  scripts/publisher_joint_trajectory_planned.py
  scripts/end_effector_pose_node.py
  scripts/send_and_monitor_trajectory.py
  scripts/sensors_orientation.py
  DESTINATION lib/${PROJECT_NAME}
)

# LAUNCH FILES -----------------------------------------------
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# YAML FILES --------------------------------------------------
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# URDF FILES -------------------------------------------------
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# RVIZ FILES -------------------------------------------------
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

#--------------------------------------------------------------------
# export dependencies
# ament_export_dependencies(${dependencies})

ament_export_dependencies(ur_robot_driver)

ament_package()
