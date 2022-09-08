cmake_minimum_required(VERSION 3.0.2)
project(ros_repo1)

## Has dependency on message_generation, std_msgs, roscpp.
## An error occurs during the build if these packages do not exist.
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    )

## Generate messages in the 'msg' folder
add_message_files(

   FILES
   Barometer.msg
   
)
add_service_files(
  FILES
  AddTwoInts.srv
)
 generate_messages(
   DEPENDENCIES
   std_msgs
)



catkin_package(
   CATKIN_DEPENDS
   message_generation
   roscpp
   rospy
   std_msgs
)


