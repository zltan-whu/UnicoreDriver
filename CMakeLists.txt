cmake_minimum_required(VERSION 3.0.2)
project(unicore_driver)

set(CMAKE_BUILD_TYPE "release")
set(CMAKE_CXX_FLAGS "-std=c++11 ")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES unicore_driver
#  CATKIN_DEPENDS gnss_bds_comm nav_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/unicore_driver.cpp                               
                               ${PROJECT_SOURCE_DIR}/src/serial_handler.cpp
                               ${PROJECT_SOURCE_DIR}/src/unicore_message_processor.cpp
                               ${PROJECT_SOURCE_DIR}/src/ntripclint_handler.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )

