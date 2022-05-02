/**
 * @file slove.h
 * @brief 
 * kalman用来简介调用kalman类 实现卡尔曼滤波 只有一个外部接口slove(int x,int t)
 * 调参在kal_test的构造函数里
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-01-17
 * 
 */

#ifndef _SLOVE_H_
#define _SLOVE_H_

#include <predictor/kalman.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <options.h>
#include <armor_finder/send_data.h>

constexpr int S=2;
class predictorKalman{
    using _Kalman = Kalman<1, S>;
    _Kalman kalman;
    Eigen::Matrix3d R_CI;           // 陀螺仪坐标系到相机坐标系旋转矩阵EIGEN-Matrix
    Eigen::Matrix3d F;              // 相机内参矩阵EIGEN-Matrix
    Eigen::Matrix<double, 1, 5> C;  // 相机畸变矩阵EIGEN-Matrix
    cv::Mat R_CI_MAT;               // 陀螺仪坐标系到相机坐标系旋转矩阵CV-Mat
    cv::Mat F_MAT;                  // 相机内参矩阵CV-Mat
    cv::Mat C_MAT;                  // 相机畸变矩阵CV-Mat

    // 相机坐标系内坐标--->世界坐标系内坐标
    inline Eigen::Vector3d pc_to_pw(const Eigen::Vector3d &pc, const Eigen::Matrix3d &R_IW) {
        auto R_WC = (R_CI * R_IW).transpose();
        return R_WC * pc;
    }

    // 世界坐标系内坐标--->相机坐标系内坐标
    inline Eigen::Vector3d pw_to_pc(const Eigen::Vector3d &pw, const Eigen::Matrix3d &R_IW) {
        auto R_CW = R_CI * R_IW;
        return R_CW * pw;
    }

    // 相机坐标系内坐标--->图像坐标系内像素坐标
    inline Eigen::Vector3d pc_to_pu(const Eigen::Vector3d &pc) {
        return  F * pc / pc(2, 0);
    }

    // 将世界坐标系内一点，投影到图像中，并绘制该点
    inline void re_project_point(cv::Mat &image, const Eigen::Vector3d &pw, const cv::Scalar &color) {
        // Eigen::Vector3d pc = pw_to_pc(pw, R_IW);
        Eigen::Vector3d pu = pc_to_pu(pw);
        cv::circle(image, {int(pu(0, 0)), int(pu(1, 0))}, 3, color, 2);
    }

    // pnp解算:获取相机坐标系内装甲板坐标
    Eigen::Vector3d pnp_get_pc(const cv::Point2f p[4], int);

public:
    predictorKalman();
    std::vector<double> predictor(double x,double t);
    bool predict(src_date &, send_data &, cv::Mat &);
    void Init(double x,double t);
};

#endif