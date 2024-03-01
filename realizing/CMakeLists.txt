cmake_minimum_required(VERSION 3.8)
PROJECT(KALMAN)

find_package(Eigen3 REQUIRED)


include_directories(KalmanFilter)
aux_source_directory(KalmanFilter KalmanFilter_src)


add_executable(Out main.cpp ${KalmanFilter_src})