cmake_minimum_required(VERSION 2.8.3)
project(referee)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
)

# Declare ROS messages
add_message_files(DIRECTORY msg FILES GameState.msg)

# Generate added messages
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

# Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime)