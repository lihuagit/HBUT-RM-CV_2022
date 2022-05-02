#include <predictor/PredictorKalman.h>
#include <math.h>

void predictorKalman::Init(double x,double t) { kalman.reset(x,t); }

predictorKalman::predictorKalman(){
    // 舒适化卡拉曼各参数
    _Kalman::Matrix_xxd A = _Kalman::Matrix_xxd::Identity(); //单位矩阵
    _Kalman::Matrix_zxd H{1,0};
    _Kalman::Matrix_xxd R;
    R(0, 0) = 0.01;
    for (int i = 1; i < S; i++) {
        R(i, i) = 100;
        // R(i, i) = 0.01;  //for dis   
    }
    _Kalman::Matrix_zzd Q{4};
    _Kalman::Matrix_x1d init{0, 0};
    kalman = _Kalman(A, H, R, Q, init, 0);

    // 初始化内参矩阵、畸变矩阵
    cv::FileStorage fin(PROJECT_DIR"/asset/camera-param.yml", cv::FileStorage::READ);
    fin["Tcb"] >> R_CI_MAT;
    fin["K"] >> F_MAT;
    fin["D"] >> C_MAT;
    cv::cv2eigen(R_CI_MAT, R_CI);
    cv::cv2eigen(F_MAT, F);
    cv::cv2eigen(C_MAT, C);
}

std::vector<double> predictorKalman::predictor(double x,double t){
    _Kalman::Matrix_z1d Zk{x};
    _Kalman::Matrix_x1d ansXk;
    ansXk=kalman.update(Zk,t);
    std::vector<double> ans(5);
    ans.emplace_back(ansXk(0,0));
    ans.emplace_back(ansXk(1,0));
    return ans;
}

bool predictorKalman::predict(src_date &data, send_data &send, cv::Mat &im2show){
    auto &[detection, rec_yaw, tag_id, t] = data;
    Eigen::Vector3d m_pc = pnp_get_pc(detection, tag_id);             // point camera: 目标在相机坐标系下的坐标
    double GUN_CAM_DISTANCE_Y = 0;
	m_pc(1, 0) -= GUN_CAM_DISTANCE_Y;
	double x_pos = m_pc(0, 0);
	double y_pos = m_pc(1, 0);
	double z_pos = m_pc(2, 0);
	double distance = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);

	double tan_pitch = y_pos / sqrt(x_pos*x_pos + z_pos * z_pos);
	double tan_yaw = x_pos / z_pos;
	double mc_pitch = -atan(tan_pitch);
    double mc_yaw = atan(tan_yaw);

    // double m_yaw=mc_yaw-rec_yaw;
    double m_yaw=mc_yaw;

    static double last_yaw = 0, last_speed = 0;
    if(std::fabs(last_yaw - m_yaw) > 5. / 180. * M_PI){
        kalman.reset(m_yaw, t);
        last_yaw = m_yaw;
        std::cout << "reset" << std::endl;
        return false;
    }

    last_yaw = m_yaw;///
    Eigen::Matrix<double, 1, 1> z_k{m_yaw};
    _Kalman::Matrix_x1d state = kalman.update(z_k, t);                        // 更新卡尔曼滤波
    last_speed = state(1, 0);
    double c_yaw = state(0, 0);                                   // current yaw: yaw的滤波值，单位弧度
    double c_speed = state(1, 0) * m_pc.norm();                      // current speed: 角速度转线速度，单位m/s

    
    double predict_time = m_pc.norm() / shoot_v + shoot_delay_t;           // 预测时间=发射延迟+飞行时间（单位:s）
    double p_yaw = c_yaw + atan2(predict_time * c_speed, m_pc.norm());     // predict yaw: yaw的预测值，直线位移转为角度，单位弧度

    double length = sqrt(m_pc(0, 0) * m_pc(0, 0) + m_pc(1, 0) * m_pc(1, 0));
    Eigen::Vector3d c_pw{length * cos(c_yaw), length * sin(c_yaw), m_pc(2, 0)};//反解位置(世界坐标系)
    Eigen::Vector3d p_pw{length * cos(p_yaw), length * sin(p_yaw), m_pc(2, 0)};
    
    distance = p_pw.norm();                          // 目标距离（单位:m）
    double distance_xy = p_pw.topRows<2>().norm();
    double p_pitch = std::atan2(p_pw(2, 0), distance_xy);
    // return true;
        // 计算抛物线
    // 先解二次方程
//    std::cout << "p_pitch: " << p_pitch <<std::endl;
    double a = 9.8 * 9.8 * 0.25;
    double b = -shoot_v * shoot_v - distance * 9.8 * cos(M_PI_2 + p_pitch);
    double c = distance * distance;
    // 带入求根公式，解出t^2
    double t_2 = (- sqrt(b * b - 4 * a * c) - b) / (2 * a);
//    std::cout << fmt::format("a:{}, b:{}, c:{}", a, b, c) << std::endl;
//    std::cout << "t2:" << t_2 << std::endl;
    double fly_time = sqrt(t_2);                                       // 子弹飞行时间（单位:s）
    // 解出抬枪高度，即子弹下坠高度
    double height = 0.5 * 9.8 * t_2;

    //std::cout << m_pw << std::endl;
    float bs = shoot_v;
    //std::cout << fmt::format("bullet_speed:{}, distance: {}, height: {}, p_pitch:{}",
    //                         bs, distance, height, p_pitch / M_PI * 180) << std::endl;
	Eigen::Vector3d s_pw{p_pw(0, 0), p_pw(1, 0), p_pw(2, 0) + height}; // 抬枪后预测点

    
    for (int i = 0; i < 4; ++i)
        cv::circle(im2show, detection[i], 3, {255, 0, 0});
    cv::circle(im2show, {im2show.cols / 2, im2show.rows / 2}, 3, {0, 255 ,0});
	re_project_point(im2show, c_pw, {0, 255, 0});
	re_project_point(im2show, p_pw, {255, 0, 0});
	re_project_point(im2show, s_pw, {0, 0, 255});
	re_project_point(im2show, m_pc, {0, 0, 255});
    return true;
}

Eigen::Vector3d predictorKalman::pnp_get_pc(const cv::Point2f p[4], int armor_number) {
    static const std::vector<cv::Point3d> pw_small = {  // 单位：m
            {-0.066, 0.027,  0.},
            {-0.066, -0.027, 0.},
            {0.066,  -0.027, 0.},
            {0.066,  0.027,  0.}
    };
    static const std::vector<cv::Point3d> pw_big = {    // 单位：m
            {-0.115, 0.029,  0.},
            {-0.115, -0.029, 0.},
            {0.115,  -0.029, 0.},
            {0.115,  0.029,  0.}
    };
    std::vector<cv::Point2d> pu(p, p + 4);
    cv::Mat rvec, tvec;

    if (armor_number == 0 || armor_number == 1 || armor_number==8)
        cv::solvePnP(pw_big, pu, F_MAT, C_MAT, rvec, tvec);
    else
        cv::solvePnP(pw_small, pu, F_MAT, C_MAT, rvec, tvec);

    Eigen::Vector3d pc;
    cv::cv2eigen(tvec, pc);
    // std::cout<<"pc:\n";
    // std::cout<<pc<<std::endl;
    pc[0] -= 0.04;
    pc[1] -= 0.01;
    // pc[2] += 0.02385;
    return pc;
}