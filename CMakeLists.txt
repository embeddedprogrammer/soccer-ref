cmake_minimum_required(VERSION 2.8.3)
project(soccerref)
add_definitions(-std=c++11)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
    image_transport
    sensor_msgs
    cv_bridge
)

# Declare ROS messages
add_message_files(DIRECTORY msg FILES GameState.msg)

# Generate added messages
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

# Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime)

# Specify additional locations of header files
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# Declare C++ executables
add_executable(soccerref_vision src/vision.cpp)
find_package( OpenCV REQUIRED )
target_link_libraries(soccerref_vision ${OpenCV_LIBS} ${catkin_LIBRARIES})