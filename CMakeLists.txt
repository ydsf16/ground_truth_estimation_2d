cmake_minimum_required(VERSION 2.6)
project(ground_truth_estimation_2d)

## Compile as C++11
add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  cv_bridge
)

find_package(OpenCV 3.0 QUIET)
find_package(Eigen3  REQUIRED)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)


## Calibration.
add_executable(calbration calbration.cpp)
target_link_libraries(calbration
${OpenCV_LIBS}
${EIGEN3_LIBS}
${catkin_LIBRARIES}
)

## Estimation.
add_executable(estimation estimation.cpp)
target_link_libraries(estimation
${OpenCV_LIBS}
${EIGEN3_LIBS}
${catkin_LIBRARIES}
)

