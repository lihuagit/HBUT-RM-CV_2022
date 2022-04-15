//
// Created by xinyang on 19-7-11.
//

#include <armor_finder/armor_finder.h>
#include <config/setconfig.h>
#include <log.h>
#include <iostream>
#include <string.h>

static bool sendTarget(Serial &serial, float x, int16_t y , double z/*, uint16_t shoot_delay*/) {
    static short x_tmp, y_tmp, z_tmp;
    uint8_t buff[10];

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

    // printf("\nx_tmp = %d, y_tmp = %d\nx = %d, y = %d\n", x_tmp, y_tmp, x, y); // for debug

    //z_tmp = static_cast<short>(z * (32768 - 1) / 1000);

    buff[0] = 's';
    // buff[1] = static_cast<char>((x_tmp >> 8) & 0xFF);
    // buff[2] = static_cast<char>((x_tmp >> 0) & 0xFF);
    // buff[3] = static_cast<char>((y_tmp >> 8) & 0xFF);
    // buff[4] = static_cast<char>((y_tmp >> 0) & 0xFF);
    printf("x:%f\n",x);
    // std::cout<<"x:"<<x<<std::endl;
    // *((float*)(buff + 1)) = x;
    float test = x;
    memcpy(buff + 1, &test, 4);

    printf("data: %f\n", *(float*)(buff + 1));

    *((uint16_t*)(buff + 5)) = y;
    *((uint16_t*)(buff + 7)) = z;



    // buff[5] = static_cast<char>((z_tmp >> 8) & 0xFF);
    // buff[6] = static_cast<char>((z_tmp >> 0) & 0xFF);
    // buff[7] = static_cast<char>((shoot_delay >> 8) & 0xFF);
    // buff[8] = static_cast<char>((shoot_delay >> 0) & 0xFF);
    buff[9] = 'e';
    // for debug
    // for(int i = 0 ; i < sizeof(buff) ; i++) {
    //     printf("%02x ", buff[i]);
    // }

    // printf("\n"); // fordebug
//    if(buff[7]<<8 | buff[8])
//        cout << (buff[7]<<8 | buff[8]) << endl;
    return serial.WriteData(buff, sizeof(buff));
}

/**
 * @brief 
 * 发送sendDate
 * 目前发送的的send_pitch为dy  send_dist不准确 后期修改
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
    return sendTarget(serial, send_yaw, (int16_t)send_pitch, (int16_t)send_dist/*, shoot_delay*/);
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

    getsystime(now_t);
    float w_yaw=c_yaw-word_yaw;
    if(fabs(send_yaw-w_yaw)<=5. / 180. * M_PI)
        send_yaw=kal_yaw.predictor(w_yaw,now_t);
    else{
        kal_yaw.Init(w_yaw,now_t);
        send_yaw=w_yaw;
    }
    dy-=50;
    send_dist=kal_dis.predictor(send_dist,0);
    send_pitch=dy;
    ////////////////////DEBUG//////////////////////
    bool debug=false;
    if(debug){
        printf("dx:%d\n",dx);
        printf("now_t:%lf\n",now_t);
        printf("c_yaw:%f\n",c_yaw);
        printf("word_yaw:%f\n",word_yaw);
        printf("send_yaw:%f\n",send_yaw);
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
    send_dist=kal_dis.predictor(send_dist,0);
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
    send_yaw=w_yaw;
    send_dist=kal_dis.predictor(send_dist,0);
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