cmake_minimum_required(VERSION 2.8.3)
project(car_hackathon)

set (CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  sensor_msgs
  car_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV 4 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(line_follower_cpp src/line_follower.cpp)
target_link_libraries(line_follower_cpp
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
add_dependencies(line_follower_cpp ${catkin_EXPORTED_TARGETS})
