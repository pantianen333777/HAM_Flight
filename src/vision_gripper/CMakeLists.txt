cmake_minimum_required(VERSION 3.0.2)
project(vision_gripper)

find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  cv_bridge
  gazebo_msgs
  geometry_msgs
  mavros_msgs
  message_filters
  nav_msgs
  nodelet
  quadrotor_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/apriltag_gripper_node.py
  scripts/calibrate_apriltag_depth.py
  scripts/gazebo_apriltag_truth_node.py
  scripts/apriltag_rtk_preflight.py
  scripts/apriltag_trajectory_mission_node.py
  scripts/vision_gripper_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_tracking_math.py)
endif()
