/**
 * @file PredictorAdaptiveEKF.h
 * @brief 相仿上交2021年代码写的EKF 目前还没优化好，没有应用
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-01-17
 * 
 */
#ifdef add_EKF

#ifndef CVRM2021_PREDICTORADAPTIVEEKF_H
#define CVRM2021_PREDICTORADAPTIVEEKF_H

#include <predictor/AdaptiveEKF.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <ceres/ceres.h>
#include <opencv2/core/eigen.hpp>
#include <options.h>


struct Predict {
    /*
     * 此处定义匀速直线运动模型
     */
    template<class T>
    void operator()(const T x0[5], T x1[5]) {
        x1[0] = x0[0] + delta_t * x0[1];  //0.1
        x1[1] = x0[1];  //100
        x1[2] = x0[2] + delta_t * x0[3];  //0.1
        x1[3] = x0[3];  //100
        x1[4] = x0[4];  //0.01
    }

    double delta_t;
};

template<class T>
void xyz2pyd(T xyz[3], T pyd[3]) {
    /*
     * 工具函数：将 xyz 转化为 pitch、yaw、distance
     */
    pyd[0] = ceres::atan2(xyz[2], ceres::sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]));  // pitch
    pyd[1] = ceres::atan2(xyz[1], xyz[0]);  // yaw
    pyd[2] = ceres::sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]+xyz[2]*xyz[2]);  // distance
}

struct Measure {
    /*
     * 工具函数的类封装
     */
    template<class T>
    void operator()(const T x[5], T y[3]) {
        T x_[3] = {x[0], x[2], x[4]};
        xyz2pyd(x_, y);
    }
};


class PredictorAdaptiveEKF {
private:
    AdaptiveEKF<5, 3> ekf;  // 创建ekf

    double last_time;

    Eigen::Matrix3d R_CI;               // 陀螺仪坐标系到相机坐标系旋转矩阵EIGEN-Matrix
    Eigen::Matrix3d F;                  // 相机内参矩阵EIGEN-Matrix
    Eigen::Matrix<double, 1, 5> C;      // 相机畸变矩阵EIGEN-Matrix
    cv::Mat R_CI_MAT;                   // 陀螺仪坐标系到相机坐标系旋转矩阵CV-Mat
    cv::Mat F_MAT;                      // 相机内参矩阵CV-Mat
    cv::Mat C_MAT;                      // 相机畸变矩阵CV-Mat
    // std::vector<bbox_t> last_boxes;     // 上一帧识别到的所有装甲板
    // bbox_t last_sbbox;                  // 上一帧击打的装甲板
    Eigen::Matrix3d last_R_IW;          // 上一帧击陀螺仪数据
    Eigen::Vector3d last_m_pw;          // 上一帧击打的装甲板的世界坐标
    double last_yaw = 0, last_pitch = 0;

public:
    inline void load_param(bool update_all = true){
        /*
         * 通过文件读入，实现实时调餐
         */
        cv::FileStorage fin(PROJECT_DIR"/asset/autoaim-param.yml", cv::FileStorage::READ);

        // 设置对角线的值
        // 预测过程协方差
        fin["Q00"] >> ekf.Q(0, 0);
        fin["Q11"] >> ekf.Q(1, 1);
        fin["Q22"] >> ekf.Q(2, 2);
        fin["Q33"] >> ekf.Q(3, 3);
        fin["Q44"] >> ekf.Q(4, 4);
        // 观测过程协方差
        fin["R00"] >> ekf.R(0, 0);
        fin["R11"] >> ekf.R(1, 1);
        fin["R22"] >> ekf.R(2, 2);
    }

    explicit PredictorAdaptiveEKF( ) {
        cv::FileStorage fin(PROJECT_DIR"/asset/camera-param.yml", cv::FileStorage::READ);
        fin["Tcb"] >> R_CI_MAT;
        fin["K"] >> F_MAT;
        fin["D"] >> C_MAT;
        cv::cv2eigen(R_CI_MAT, R_CI);
        cv::cv2eigen(F_MAT, F);
        cv::cv2eigen(C_MAT, C);

        load_param();

        std::cout << "Finish create a new EKF." << std::endl;
    }

    Eigen::Matrix<double, 5, 1> predict(double m_yaw,double m_pitch,double m_dist);
    bool Init(double m_yaw,double m_pitch,double m_dist);

    ~PredictorAdaptiveEKF() = default;
};


#endif //CVRM2021_PREDICTORADAPTIVEEKF_H
#endif //add_EKF
