cmake_minimum_required(VERSION 2.8.3)
project(image_cut)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    cv_bridge
    message_filters
)

find_package(OpenCV REQUIRED)
find_package(Eigen3)
    
catkin_package(
  INCLUDE_DIRS 
  DEPENDS PCL
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIR}
)


add_executable(image_cut src/image_cut.cpp)
add_dependencies(image_cut ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_cut ${catkin_LIBRARIES} ${OpenCV_LIBS}  )

