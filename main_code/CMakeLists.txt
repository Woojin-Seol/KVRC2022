cmake_minimum_required(VERSION 3.19.0)
project(khnp_competition)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14") # if want to use auto in Lamda function, should be more recent than c++14
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(Qt5 REQUIRED COMPONENTS
  Core Widgets 
)
if(Qt5_FOUND)
  message(WARNING "Qt5 FOUND")
  message(WARNING ${Qt5_VERSION})
  message(WARNING ${Qt5_DIR})
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  geometry_msgs
  sensor_msgs
  gazebo_msgs
  rosgraph_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)
if(OPENCV_FOUND)
  message(WARNING "OpenCV FOUND")
  message(WARNING ${OpenCV_VERSION})
  message(WARNING ${OpenCV_INCLUDE_DIRS})
endif()

catkin_package()

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

#### Scoring code
add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS} Qt5::Core Qt5::Widgets)