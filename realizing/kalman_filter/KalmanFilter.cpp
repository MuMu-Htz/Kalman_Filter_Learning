#include <iostream>
#include "KalmanFilter.hpp"


KalmanFilter::KalmanFilter(Eigen::VectorXd Vec_x0, Eigen::MatrixXd Mat_P0, int n = 2)
{
    this->resize_Mat_(n); //类型选择
    this->init_Mat_2(); //矩阵元素初始化

    this->Vec_hatx_k_before_ = Vec_x0;
    this->Mat_P_k_1_ = Mat_P0;

}
KalmanFilter::KalmanFilter(Eigen::VectorXd Vec_x0, Eigen::MatrixXd Mat_P0, std::vector<double> num_w, std::vector<double> num_v, int n = 2)
{
    this->resize_Mat_(n); //类型选择
    this->init_Mat_2(); //矩阵元素初始化

    this->Vec_hatx_k_before_ = Vec_x0;
    this->Mat_P_k_1_ = Mat_P0;

    this->init_Mat_Q(num_w);
    this->init_Mat_R(num_v);

}
KalmanFilter::~KalmanFilter()
{
    
}


/**
 * @brief 矩阵元素的初始化
 * @param 
 * @return none
*/
void KalmanFilter::init_Mat_2()
{

    this->Mat_A_ << 1, 1,
                    0, 1;
    this->Mat_R_ << 1, 0,
                    0, 1;
    this->Mat_H_ << 1, 1,
                    0, 1;
    this->Mat_I_ << 1, 0,
                    0, 1;
    this->Mat_Q_ << 0.1, 0,
                    0, 0.1;
}


/**
 * @brief 确定所有方阵的阶数
 * @param n 方阵的阶数
 * @return none
*/
void KalmanFilter::resize_Mat_(int n)
{
    this->Mat_A_.resize(n, n);
    this->Mat_H_.resize(n, n);
    this->Mat_I_.resize(n, n);
    this->Mat_Kk_.resize(n, n);
    this->Mat_P_k_1_.resize(n, n);
    this->Mat_Q_.resize(n, n);
    this->Mat_R_.resize(n ,n);

    this->Vec_hatx_k_1_.resize(1, n);
    this->Vec_hatx_k_.resize(1, n);
    this->Vec_hatx_k_before_.resize(1, n);
    this->Vec_Z_k_.resize(1, n);
}

/**
 * @brief 先验估计 
 * @param 
 * @return none
*/
void KalmanFilter::priori_estimates()
{
    this->Vec_hatx_k_before_ = this->Mat_A_ * this->Vec_hatx_k_1_;
}

/**
 * @brief 先验误差协方差
 * @param
 * @return none
*/
void KalmanFilter::priori_error_covariance()
{
    this->Mat_P_k_before_ = 
    this->Mat_A_ * this->Mat_P_k_1_ * this->Mat_A_.transpose() + this->Mat_Q_;
}

/**
 * @brief 计算卡尔曼增益
 * @param
 * @return none
*/
void KalmanFilter::kalman_gain()
{
    Eigen::MatrixXd Mat_inner = this->Mat_Kk_;
    Mat_inner = this->Mat_H_ * this->Mat_P_k_before_ * this->Mat_H_.transpose() + this->Mat_R_;
    this->Mat_Kk_ = (this->Mat_P_k_before_ * this->Mat_H_.transpose()) * Mat_inner.inverse();
}

/**
 * @brief 计算后验估计
 * @param Mat_Z 观测矩阵
 * @return none
*/
Eigen::VectorXd KalmanFilter::posterior_estimates(Eigen::VectorXd Vec_Z)
{
    this->Vec_hatx_k_ = this->Vec_hatx_k_before_ + this->Mat_Kk_ * (Vec_Z - this->Mat_H_ * this->Vec_hatx_k_before_);

    this->Vec_hatx_k_1_ = this->Vec_hatx_k_; //更新hatx{k-1}

}

/**
 * @brief 更新误差协方差矩阵
 * @param
 * @return none
*/
void KalmanFilter::update_error_covariance()
{
    this->Mat_P_k_ = (this->Mat_I_ - this->Mat_Kk_ * this->Mat_H_) * this->Mat_P_k_before_;
    this->Mat_P_k_1_ = this->Mat_P_k_;
}

/**
 * @brief 更新下一个状态
 * @param Mat_Z 观测矩阵
 * @return none
*/
void KalmanFilter::update(Eigen::VectorXd Vec_z, Eigen::VectorXd & Vec_hatx_k_)
{
    this->priori_estimates();
    this->priori_error_covariance();
    this->kalman_gain();
    Vec_hatx_k_ = this->posterior_estimates(Vec_z);
    this->update_error_covariance();
}

void KalmanFilter::init_Mat_Q(std::vector<double> nums)
{
    for(int i = 0; i < nums.size(); i++)
    {
        this->Mat_Q_ << nums[i];
    }
}

void KalmanFilter::init_Mat_R(std::vector<double> nums)
{
    for(int i = 0; i < nums.size(); i++)
    {
        this->Mat_R_ << nums[i];
    }
}

