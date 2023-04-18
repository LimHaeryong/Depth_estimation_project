cmake_minimum_required(VERSION 3.0.2)
project(tstl_depth)

set (OpenCV_DIR /usr/share/OpenCV)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  xycar_msgs
)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV 4.5.5 REQUIRED PATHS ~/OpenCV4/install/lib/cmake/opencv4)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library(modules1
  src/${PROJECT_NAME}/tstl_depth.cpp
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
  modules1
  ${YAML_CPP_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}  
)
