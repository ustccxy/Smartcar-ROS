cmake_minimum_required(VERSION 2.8.3)
project(location_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
    FILES
    UtmWithPose.msg
    UtmPoint.msg
    UtmOdometer.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
)


catkin_package(
   CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime

)
