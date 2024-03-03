#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <vector>
class KalmanFilter
{
private:

Eigen::MatrixXd Mat_A_; //状态矩阵
Eigen::MatrixXd Mat_R_; //v的协方差矩阵
Eigen::MatrixXd Mat_Q_; //w的协方差矩阵
Eigen::MatrixXd Mat_H_; //测量状态矩阵
Eigen::MatrixXd Mat_I_; //单位矩阵


Eigen::VectorXd Vec_hatx_k_before_; //k时刻的先验估计值
Eigen::VectorXd Vec_hatx_k_1_; //k-1时刻的后验估计
Eigen::MatrixXd Mat_P_k_before_; //k时刻先验误差协方差矩阵
Eigen::MatrixXd Mat_P_k_1_; //k-1时刻的误差协方差矩阵
Eigen::MatrixXd Mat_Kk_; //卡尔曼增益
Eigen::VectorXd Vec_hatx_k_; //k时刻的后验估计
Eigen::VectorXd Vec_Z_k_; //k时刻的测量向量
Eigen::MatrixXd Mat_P_k_; //k时刻的误差协方差矩阵&也是k+1时刻的k-1时刻的误差协方差矩阵

void init_Mat_2();
void resize_Mat_(int n);

//卡尔曼循环
void priori_estimates();
void priori_error_covariance();
void kalman_gain();
Eigen::VectorXd posterior_estimates(Eigen::VectorXd Vec_z);
void update_error_covariance();



public:

KalmanFilter(Eigen::VectorXd Vec_x0, Eigen::MatrixXd Mat_P0, int n = 2);
~KalmanFilter();
void update(Eigen::VectorXd Vec_z, Eigen::VectorXd & Vec_hatx_k_);
void init_Mat_R(std::vector<double> nums);
void init_Mat_Q(std::vector<double> nums);
};

