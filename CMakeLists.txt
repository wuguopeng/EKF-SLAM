cmake_minimum_required(VERSION 2.8.3)
project(myekf)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
#设置路径(Eigen所在的路径，也就是编译的路径）
set(Eigen_INCLUDE_DIRS /usr/include/eigen3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cmake_modules
  nav_msgs
  tf
  geometry_msgs
  visualization_msgs
)

# 寻找Eigen库
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include 
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs tf geometry_msgs visualization_msgs
#  DEPENDS system_lib
)

# 添加头文件
include_directories( ${Eigen_INCLUDE_DIRS} )

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(myekf src/main.cpp src/ekfslam.cpp)

add_dependencies(myekf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(myekf
   ${catkin_LIBRARIES} ${OpenCV_LIBS}
)
