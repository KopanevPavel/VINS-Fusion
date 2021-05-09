cmake_minimum_required(VERSION 2.8.3)
project(dnn_fusion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

add_subdirectory(./ThirdParty/json/)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ./ThirdParty/json/include/
)

catkin_package()

add_executable(dnn_fusion_node 
	src/dnnOptNode.cpp)
	# src/dnnOpt.cpp)

target_link_libraries(dnn_fusion_node ${OpenCV_LIBS} ${catkin_LIBRARIES} ${CERES_LIBRARIES}) 
