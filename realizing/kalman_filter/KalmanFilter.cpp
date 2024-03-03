#include <iostream>
#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(Eigen::VectorXd Vec_x0, Eigen::MatrixXd Mat_P0, int n /* = 2*/)
{
    this->resize_Mat_(n); //类型选择
    this->init_Mat_2(); //矩阵元素初始化

    this->Vec_hatx_k_before_ = Vec_x0;
    this->Mat_P_k_1_ = Mat_P0;

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

    this->Vec_hatx_k_1_.resize(n);
    this->Vec_hatx_k_.resize(n);
    this->Vec_hatx_k_before_.resize(n);
    this->Vec_Z_k_.resize(n);
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
    return Vec_hatx_k_;
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

/**
 * @brief 初始化Q矩阵的函数 主要用于外部更改private中的成员函数
 * @param nums 将数据以vector形式的输入
 * @return none
*/
void KalmanFilter::init_Mat_Q(std::vector<double> nums)
{
    int rows = this->Mat_Q_.rows();
    int cols = this->Mat_Q_.cols();
    if(nums.size() != rows * cols)
    {
        std::cout << "Error: The size of the vector does not match the size of the matrix!\n";
        return;
    }
    for(int i = 0; i < rows; ++i)
    {
        for(int j = 0; j < cols; ++j)
        {
            this->Mat_Q_(i, j) = nums[i * cols + j];
        }
    }
}

/**
 * @brief 初始化R矩阵的函数 主要用于外部更改private中的成员函数
 * @param nums 将数据以vector形式的输入
 * @return none
*/
void KalmanFilter::init_Mat_R(std::vector<double> nums)
{
    int rows = this->Mat_R_.rows();
    int cols = this->Mat_R_.cols();
    if(nums.size() != rows * cols)
    {
        std::cout << "Error: The size of the vector does not match the size of the matrix!\n";
        return;
    }
    for(int i = 0; i < rows; ++i)
    {
        for(int j = 0; j < cols; ++j)
        {
            this->Mat_R_(i, j) = nums[i * cols + j];
        }
    }
}

