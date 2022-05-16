/**
 * @file PredictorAdaptiveEKF.cpp
 * @brief 根据上交2021年代码写的EKF，但是没有优化好，没有应用
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-05-16
 * 
 */
#ifdef add_EKF

#include <predictor/PredictorAdaptiveEKF.h>


////////////////////////// 调参区 start //////////////////////////





////////////////////////// 调参区 end //////////////////////////


struct LastTimeHelper {
    LastTimeHelper(double current_time, double &ref_time) : c_time(current_time), r_time(ref_time) {};

    ~LastTimeHelper() { r_time = c_time; }

    double c_time;
    double &r_time;
};

Eigen::Matrix<double, 5, 1> PredictorAdaptiveEKF::predict(double m_yaw,double m_pitch,double m_dist){
    double shoot_delay;
    double last_yaw = m_yaw;
    double last_pitch = m_pitch;

    Predict predictfunc;
    Measure measure;

    Eigen::Matrix<double, 3, 1> Yr={m_yaw,m_pitch,m_dist};
    predictfunc.delta_t = shoot_delay_t;      // 设置距离上次预测的时间
    ekf.predict(predictfunc);           // 更新预测器，此时预测器里的是预测值
    Eigen::Matrix<double, 5, 1> Xe = ekf.update(measure, Yr);   // 更新滤波器，输入真实的球面坐标 Yr
    double predict_time = shoot_delay;   // 预测时间=发射延迟+飞行时间（单位:s）
    predictfunc.delta_t = shoot_delay_t;     // 设置本次预测时间
    Eigen::Matrix<double, 5, 1> Xp;
    predictfunc(Xe.data(), Xp.data());      // 使用匀速直线模型直接预测 Xp
    Eigen::Vector3d c_pw{Xe(0, 0), Xe(2, 0), Xe(4, 0)};
    Eigen::Vector3d p_pw{Xp(0, 0), Xp(2, 0), Xp(4, 0)};
    return Xp;
}

bool PredictorAdaptiveEKF::Init(double m_yaw,double m_pitch,double m_dist){
    Eigen::Matrix<double, 5, 1> Xr;
    Xr << m_yaw, 0, m_pitch, 0, m_dist;
    ekf.init(Xr);
    last_yaw = m_yaw;
    last_pitch = m_pitch;
    return true;
}

#endif  //add_EKF