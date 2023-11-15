# MIRA needs cmake >= 2.8
cmake_minimum_required(VERSION 2.8.3)
project(gazebolistener)

# include(${ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  cv_bridge
  sensor_msgs
  theora_image_transport
  nav_msgs
  tf2_msgs
)

find_package(OpenCV 3.4.13 REQUIRED)

include_directories(
  ${OPENCV_IMGPROC_LIBRARY}
  ${catkin_INCLUDE_DIRS}
  /opt/ros/noetic/include  # Add the ROS include directory
  ${CMAKE_CURRENT_LIST_DIR}/include
)

# rosbuild_init()

# include cmake file for MIRA
include(CMakeLists.mira)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(gazebolistener 
  src/GazeboListener.cpp
  src/converter/MIRAAdapter.cpp
  src/converter/RobotStateGazebo.cpp
)
target_link_libraries(gazebolistener
  ${catkin_LIBRARIES}
  /opt/ros/noetic/lib  # Add the ROS library directory
  -W  # Add the -W flag
)

# for mira support (auto linking, enabling C++0x support,...)
rosbuild_add_mira(gazebolistener)
