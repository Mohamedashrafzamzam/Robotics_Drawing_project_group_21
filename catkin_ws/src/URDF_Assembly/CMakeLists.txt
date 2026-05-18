cmake_minimum_required(VERSION 3.0.2)
project(URDF_Assembly)

## Find packages (add all necessary dependencies here)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs

  # Dependencies for Controllers and Gazebo
  controller_manager
  effort_controllers
  joint_state_controller
  gazebo_ros
  gazebo_ros_control
  robot_state_publisher
  xacro
)

catkin_package(
  # Include directories to be exported to other packages
  # INCLUDE_DIRS include
  # Libraries to be exported to other packages
  # LIBRARIES URDF_Assembly
  CATKIN_DEPENDS
    roscpp
    rospy
    
    controller_manager
    effort_controllers
    joint_state_controller
    gazebo_ros
    gazebo_ros_control
    robot_state_publisher
    xacro
  # The following are used for install purposes
  # DEPENDS system_lib
)

###########
## INSTALLATION
###########

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install configuration files (controllers)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install URDF files and their dependencies (meshes, etc.)
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install Python scripts
# Note: move_joint_r.py is the file we created earlier
install(PROGRAMS scripts/move_joint_r.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
