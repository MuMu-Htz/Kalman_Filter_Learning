#include <iostream>
#include <random>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "./kalman_filter/KalmanFilter.hpp"

std::vector<double> real_x;
std::vector<double> measurement_x;
std::vector<double> filtered_x;
std::vector<double> real_v;
std::vector<double> measurement_v;
std::vector<double> filtered_v;

std::random_device rd;
std::mt19937 gen(rd());

double x = 0.0;
double v = 2.0;

double w1, w2, v1, v2;
int list_size = 0;
double x0[4] = {0};
double P0[4] = {0};


Eigen::Vector2d Vec_x0;
Eigen::Matrix2d Mat_P0;
Eigen::Vector2d Vec_Z;

void init_param()
{
    cv::FileStorage fs("../settings.xml", cv::FileStorage::READ);
    fs["value_list_size"] >> list_size;
    if(!fs.isOpened())
    {
        std::cout << "Failed tp open .xml file!" << std::endl;
    }
    fs["value_w1"] >> w1;
    fs["value_w2"] >> w2;
    fs["value_v1"] >> v1;
    fs["value_v2"] >> v2;

    fs["x0_00"] >> x0[0];
    fs["x0_01"] >> x0[1];
    fs["x0_10"] >> x0[2];
    fs["x0_11"] >> x0[3];
    fs["P0_00"] >> P0[0];
    fs["P0_01"] >> P0[1];
    fs["P0_10"] >> P0[2];
    fs["P0_11"] >> P0[3];

    Vec_x0 << x0[0], x0[1];
    Mat_P0 << P0[0], P0[1],
              P0[2], P0[3];
}

void update_real(double w1, double v1)
{
    std::normal_distribution<>dis_w1(0.0, w1);
    std::normal_distribution<>dis_v1(0.0, v1);

    double real_rd_x = dis_w1(gen);
    double real_rd_v = dis_v1(gen);
    double temp_v = v;
    double temp_x = x;

    v += real_rd_v;
    x = x + v + real_rd_x;

    real_x.push_back(x);
    real_v.push_back(v);

    v = temp_v;
    x = temp_x;
}

void update_measurement(double w2, double v2)
{
    std::normal_distribution<>dis_w2(0.0, w2);
    std::normal_distribution<>dis_v2(0.0, v2); 

    double meas_rd_x = dis_w2(gen);
    double meas_rd_v = dis_v2(gen);
    double temp_v = v;
    double temp_x = x;

    v += meas_rd_v;
    x = x + v + meas_rd_x;

    measurement_x.push_back(x);
    measurement_v.push_back(v);
    
    Vec_Z << x, v;

    v = temp_v;
    x = temp_x;
}

int main()
{
    init_param();
    
    std::vector<double> Q_sigma;
    std::vector<double> R_v;
    Q_sigma.push_back(w1*w1);
    Q_sigma.push_back(w1*w2);
    Q_sigma.push_back(w2*w1);
    Q_sigma.push_back(w2*w2);
    R_v.push_back(v1*v1);
    R_v.push_back(v1*v2);
    R_v.push_back(v2*v1);
    R_v.push_back(v2*v2);
    KalmanFilter KF(Vec_x0, Mat_P0);

    KF.init_Mat_Q(Q_sigma);
    KF.init_Mat_R(R_v);
    Eigen::VectorXd Vec_hatx;
    for(int i = 0; i < list_size; i++)
    {
        update_real(w1,v1);
        update_measurement(w2,v2);
        KF.update(Vec_Z, Vec_hatx);
        filtered_x.push_back(Vec_hatx(0));
        filtered_v.push_back(Vec_hatx(1));
        x += v;

    }

    std::cout << "real_x: ";
    for(int i = 0; i < list_size; i++)
    {
        std::cout << real_x[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "real_v: ";
    for(int i = 0; i < list_size; i++)
    {
        std::cout << real_v[i] << " ";
    }
    std::cout << std::endl;
    std::cout << "measurement_x: ";
    for(int i = 0; i < list_size; i++)
    {
        std::cout << measurement_x[i] << " "; 
    }
    std::cout << std::endl;
    std::cout << "measurement_v: ";
    for(int i = 0; i < list_size; i++)
    {
        std::cout << measurement_v[i] << " "; 
    }
    std::cout << std::endl;
    std::cout << "filtered_x: ";
    for(int i = 0; i < list_size; i++)
    {
        std::cout << filtered_x[i] << " "; 
    }
    std::cout << std::endl;
    std::cout << "filtered_v: ";
    for(int i = 0; i < list_size; i++)
    {
        std::cout << filtered_v[i] << " "; 
    }
    std::cout << std::endl;
}