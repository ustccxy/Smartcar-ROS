cmake_minimum_required(VERSION 2.8.3)
project(track_msgs)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  can_msgs
  message_generation
)


add_message_files(
    FILES
    LaneSection.msg
    ReplaySection.msg
    SiteSection.msg
    SectionTaskState.msg
    Station.msg
    TrjSection.msg
    WayPoint.msg
    TaskShutdown.msg
    TrackingLineSignal.msg
)


generate_messages(
    DEPENDENCIES
    std_msgs
    can_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES track_msgs
   CATKIN_DEPENDS roscpp rospy std_msgs can_msgs message_runtime
#  DEPENDS system_lib
)
