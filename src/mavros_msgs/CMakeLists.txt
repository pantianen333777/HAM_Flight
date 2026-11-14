cmake_minimum_required(VERSION 3.0.2)
project(mavros_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs)

add_message_files(FILES
  AttitudeTarget.msg
  EstimatorStatus.msg
  ExtendedState.msg
  GPSRAW.msg
  ParamValue.msg
  PositionTarget.msg
  RCIn.msg
  State.msg
)
add_service_files(FILES
  CommandBool.srv
  CommandLong.srv
  ParamGet.srv
  ParamPull.srv
  ParamPush.srv
  ParamSet.srv
  SetMode.srv
)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs)
