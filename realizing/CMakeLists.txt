cmake_minimum_required(VERSION 3.8)
PROJECT(KALMAN)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(KalmanFilter)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
aux_source_directory(KalmanFilter KalmanFilter_src)


add_executable(Out main.cpp ${KalmanFilter_src})

target_link_libraries(Out ${OpenCV_LIBS})