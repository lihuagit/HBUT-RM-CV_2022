/**
 * @file send_target.cpp
 * @brief 串口发送数据文件
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-05-16
 * 
 */

#include <armor_finder/armor_finder.h>
#include <config/setconfig.h>
#include <log.h>
#include <iostream>
#include <string.h>

static bool sendTarget(Serial &serial, float x, float y /*, double z, uint16_t shoot_delay*/) {
    static short x_tmp, y_tmp, z_tmp;
    uint8_t buff[10];

// 显示fps
#ifdef WITH_COUNT_FPS
    static time_t last_time = time(nullptr);
    static int fps;
    time_t t = time(nullptr);
    if (last_time != t) {
        last_time = t;
        std::cout << "Armor: fps:" << fps << ", (" << x << "," << y << ")" << std::endl;
        fps = 0;
    }
    fps += 1;
#endif

    x_tmp = static_cast<short>(x * (32768 - 1) / 100);
    y_tmp = static_cast<short>(y * (32768 - 1) / 100);

    buff[0] = 's';
    // printf("x:%f\n",x);
    float test = x;
    memcpy(buff + 1, &test, 4);
    test = y;
    memcpy(buff + 5, &test, 4);
    // printf("data: %f\n", *(float*)(buff + 1));
    buff[9] = 'e';
    return serial.WriteData(buff, sizeof(buff));
}

/**
 * @brief 
 * 发送sendDate
 * 目前发送的的send为pitch为dy  send_dist不准确 后期修改
 * @param shoot_delay 
 * @return true 
 * @return false 
 */
bool ArmorFinder::sendBoxPosition(uint16_t shoot_delay) {
    if(is_predictor){
        #ifdef add_EKF
        if(is_predictorEKF) updateSendDateEKF();
        else
        #endif
        if(is_predictorKalman) updateSendDateKalman();
        else updateSendDate();
    }else updateSendDate();
    return sendTarget(serial, sendData.send_yaw, sendData.send_pitch/*, (int16_t)sendData.send_dist, shoot_delay*/);
}

/**
 * @brief 
 * 基于kalman
 * 对装甲板数据进行处理转化为yaw、pitch
 * 并更新发送数据sendDate
 * @return true 
 * @return false 
 */
bool ArmorFinder::updateSendDateKalman(){

    getsystime(now_t);
    src_date data;
    send_data s_data;

    data.id=target_box.id;
    data.now_t=now_t;
    data.rec_yaw=word_yaw;
    // std::cout<<"word_yaw"<<std::endl;
    // std::cout<<word_yaw<<std::endl;
    for(int i=0;i<4;i++)
        data.p[i]=target_box.pts[i];
    double w=im2show.size().width;
    double h=im2show.size().height;
    w/=2; h/=2;
    for(int i=0;i<4;i++){
        data.p[i].x-=w;
        data.p[i].y-=h;
        data.p[i].y*=-1;
    }
    kal_yaw.predict(data,sendData,im2show);
    cv::imshow("kalman",im2show);
    return true;


    static predictorKalman kal_dis;
    if (target_box.rect == cv::Rect2d())
    return false;
    
    int16_t dx = target_box.rect.x + target_box.rect.width / 2 - IMAGE_CENTER_X;
    int16_t dy = -(target_box.rect.y + target_box.rect.height / 2 - IMAGE_CENTER_Y);
    float c_dist = DISTANCE_HEIGHT / target_box.rect.height*0.666666;   // 单位 cm

    // 对距离进行滤波
    while (Dis.size() < 6)
    {
        Dis.push(c_dist);
    }
    if (Dis.size() >= 6)
    {
        Dis.push(c_dist);
    }
    LinearSmooth72(Dis, 7);
    c_dist=Dis.front();
    Dis.pop();
    
    // 计算yaw，pitch
    double p_dist=c_dist*3; // 实际距离 转 像素距离
    float c_yaw = atan(1.0*dx / p_dist);
    float c_pitch = atan(1.0*dy / p_dist);                         // 单位 弧度 ： 目前未使用

    // 对yaw预测
    getsystime(now_t);
    float w_yaw=c_yaw-word_yaw;             // 相机坐标系转世界坐标系 ： 简单的叠加 发送的数据也是世界坐标
    vector<double> kal_res;

    // yaw轴变化幅度大于10度认为发生目标切换 重置滤波器
    // 滤波返回值kal_res{ (yaw)位置 ， (v)速度 }
    if(fabs(sendData.send_yaw-w_yaw)<=10. / 180. * M_PI)
        kal_res=kal_yaw.predictor(w_yaw,now_t);
    else{
        kal_yaw.Init(w_yaw,now_t);
        kal_res.emplace_back(w_yaw);
        kal_res.emplace_back(0);
    }

    // 预测时间=发射延迟+飞行时间（单位:s）
    double predict_t;
    predict_t=( (c_dist/100)/shoot_v ) + shoot_delay_t;
    sendData.send_yaw=kal_res[0]+kal_res[1]*predict_t;
    // send_dist=kal_dis.predictor(send_dist,0);    // 使用kalman对距离滤波 ： 已弃用
    sendData.send_dist=c_dist;
    sendData.send_pitch=dy;
    ////////////////////DEBUG//////////////////////
    bool debug=false;
    if(debug){
        printf("dx:%d\n",dx);
        printf("now_t:%lf\n",now_t);
        printf("c_yaw:%f\n",c_yaw);
        printf("word_yaw:%f\n",word_yaw);
        printf("send_yaw:%f\n",sendData.send_yaw);
    }
    ////////////////////DEBUG/////////////////////
    return true;

    // std::cout<<"dist:"<<dist<<std::endl;
    // dx+=20;
    // double dist_pre=dist;
    // if(dist_pre<70) dist_pre=50;
    // else if(dist_pre>=70 && dist_pre<=150) dist_pre=150;
    // else if(dist_pre>=150 && dist_pre<250) dist_pre=250;
    // else if(dist_pre>=250 && dist_pre<350) dist_pre=350;
    // else if(dist_pre>=350 && dist_pre<450) dist_pre=450;
    // else if(dist_pre>=450 && dist_pre<550) dist_pre=550;
    // else if(dist_pre>=550 && dist_pre<=650) dist_pre=650; 
    // double dt=dist_pre/800;
    // std::cout<<dt<<std::endl;
    // dy+=(dt*dt*980)/2;

    // if(dist>300)
    //     dy+=100;     // for 5m
    // else dy+=20;         // for 2m
    // dy+=40;
    // static int cnt=0;
    // static bool flag=true;
    // if(!flag) return true;
    // if(cnt<7){
    //     cnt++;
    //     return true;
    // }
    // // flag=false;
    // cnt=0;
    // getsystime(now_t);
    // kal_yaw.slove(dx,now_t);
    // std::cout<<yaw<<" "<<pitch<<std::endl;
    // return sendTarget(serial, dx, dy, dist/*, shoot_delay*/);
}


#ifdef add_EKF

/**
 * @brief 
 * 基于EKF
 * 对装甲板数据进行处理转化为yaw、pitch
 * 并更新数据sendDate
 * @return true 
 * @return false 
 */
bool ArmorFinder::updateSendDateEKF(){
    // 对于距离滤波还是使用kalman
    static predictorKalman kal_dis;
    if (target_box.rect == cv::Rect2d())
        return false;
    
    int16_t dx = target_box.rect.x + target_box.rect.width / 2 - IMAGE_CENTER_X;
    int16_t dy = -(target_box.rect.y + target_box.rect.height / 2 - IMAGE_CENTER_Y);
    // double yaw = atan(1.0*dx / FOCUS_PIXAL) / 180 * PI;
    // double pitch = atan(1.0*dy / FOCUS_PIXAL) / 180 * PI;
    float c_yaw = atan(1.0*dx / FOCUS_PIXAL);
    float c_pitch = atan(1.0*dy / FOCUS_PIXAL);
    float c_dist = DISTANCE_HEIGHT / target_box.rect.height*0.666666;

    float w_yaw=c_yaw-word_yaw;
    Eigen::Matrix<double, 5, 1> Xp;
    if(fabs(send_yaw-w_yaw)<=5. / 180. * M_PI){
        Xp=ekf.predict(w_yaw,c_pitch,c_dist);
        send_yaw=Xp(0,0);
    }
    else{
        ekf.Init(w_yaw,c_pitch,c_dist);
        send_yaw=w_yaw;
    }
    dy-=50;
    // send_dist=kal_dis.predictor(send_dist,0);
    send_pitch=dy;
    ////////////////////DEBUG//////////////////////
    bool debug=false;
    if(debug){
        printf("dx:%d\n",dx);
        printf("c_yaw:%f\n",c_yaw);
        printf("word_yaw:%f\n",word_yaw);
        printf("send_yaw:%f\n",send_yaw);
    }
    ////////////////////DEBUG/////////////////////
    return true;
}
#endif // add_EKF

/**
 * @brief 
 * 不适用预测
 * 对装甲板数据进行处理转化为yaw、pitch
 * 并更新数据sendDate
 * @return true 
 * @return false 
 */
bool ArmorFinder::updateSendDate(){
    // 对于距离滤波还是使用kalman
    static predictorKalman kal_dis;
    if (target_box.rect == cv::Rect2d())
        return false;
    
    int16_t dx = target_box.rect.x + target_box.rect.width / 2 - IMAGE_CENTER_X;
    int16_t dy = -(target_box.rect.y + target_box.rect.height / 2 - IMAGE_CENTER_Y);
    // double yaw = atan(1.0*dx / FOCUS_PIXAL) / 180 * PI;
    // double pitch = atan(1.0*dy / FOCUS_PIXAL) / 180 * PI;
    float c_yaw = atan(1.0*dx / FOCUS_PIXAL);
    float c_pitch = atan(1.0*dy / FOCUS_PIXAL);
    float c_dist = DISTANCE_HEIGHT / target_box.rect.height*0.666666;

    float w_yaw=c_yaw-word_yaw;
    Eigen::Matrix<double, 5, 1> Xp;
    dy-=50;
    sendData.send_yaw=w_yaw;
    // send_dist=kal_dis.predictor(send_dist,0);
    sendData.send_pitch=dy;
    ////////////////////DEBUG//////////////////////
    bool debug=false;
    if(debug){
        printf("dx:%d\n",dx);
        printf("c_yaw:%f\n",c_yaw);
        printf("word_yaw:%f\n",word_yaw);
        printf("send_yaw:%f\n",sendData.send_yaw);
    }
    ////////////////////DEBUG/////////////////////
    return true;
}

/**
 * @brief 距离判断误差很大，加入使用队列进行滤波
 * @param Input 
 * @param size 
 */
void ArmorFinder::LinearSmooth72(queue<double> &Input, int size)
{
    // if (Input.size() < 7)
    // {
    //     return;
    // }
    double input[7];
    for (int i = 0; i < size; i++)
    {
        input[i] = Input.front();
        Input.pop();
    }
    double *output = new double[size];
    long i(0);

    if (size < 7)
    {
        for (i = 0; i <= size - 1; i++)
        {
            output[i] = input[i];
        }
    }
    else
    {
        output[0] = (32.0 * input[0] + 15.0 * input[1] + 3.0 * input[2] - 4.0 * input[3] -
                     6.0 * input[4] - 3.0 * input[5] + 5.0 * input[6]) /
                    42.0;
        output[1] = (5.0 * input[0] + 4.0 * input[1] + 3.0 * input[2] + 2.0 * input[3] +
                     input[4] - input[6]) /
                    14.0;
        output[2] = (1.0 * input[0] + 3.0 * input[1] + 4.0 * input[2] + 4.0 * input[3] +
                     3.0 * input[4] + 1.0 * input[5] - 2.0 * input[6]) /
                    14.0;
        for (i = 3; i <= size - 4; i++)
        {
            output[i] = (-2.0 * (input[i - 3] + input[i + 3]) +
                         3.0 * (input[i - 2] + input[i + 2]) +
                         6.0 * (input[i - 1] + input[i + 1]) + 7.0 * input[i]) /
                        21.0;
        }
        output[size - 3] = (1.0 * input[size - 1] + 3.0 * input[size - 2] + 4.0 * input[size - 3] +
                            4.0 * input[size - 4] + 3.0 * input[size - 5] + 1.0 * input[size - 6] - 2.0 * input[size - 7]) /
                           14.0;
        output[size - 2] = (5.0 * input[size - 1] + 4.0 * input[size - 2] + 3.0 * input[size - 3] +
                            2.0 * input[size - 4] + input[size - 5] - input[size - 7]) /
                           14.0;
        output[size - 1] = (32.0 * input[size - 1] + 15.0 * input[size - 2] + 3.0 * input[size - 3] -
                            4.0 * input[size - 4] - 6.0 * input[size - 5] - 3.0 * input[size - 6] + 5.0 * input[size - 7]) /
                           42.0;
    }

    for (i = 0; i < 7; i++)
    {
        // input[i] = output[i];
        Input.push(output[i]);
    }

    delete output;
}