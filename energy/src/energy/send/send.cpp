//
// Created by xixiliadorabarry on 1/24/19.
//
#include "energy/energy.h"
#include <iostream>
#include "log.h"
#include "config/setconfig.h"

using namespace std;

#define MINMAX(value, min, max) value = ((value) < (min)) ? (min) : ((value) > (max) ? (max) : (value))

//----------------------------------------------------------------------------------------------------------------------
// 此函数用于发送能量机关数据
// ---------------------------------------------------------------------------------------------------------------------
void Energy::sendEnergy() {
    // sum_yaw += yaw_rotation;
    // sum_pitch += pitch_rotation;
    // float yaw_I_component = YAW_AIM_KI * sum_yaw;
    // float pitch_I_component = PITCH_AIM_KI * sum_pitch;
    // MINMAX(yaw_I_component, -2, 2);
    // MINMAX(pitch_I_component, -2, 2);

    // double tmp_yaw = yaw_rotation;
    // double tmp_pitch = pitch_rotation;
    // yaw_rotation = YAW_AIM_KP * yaw_rotation + YAW_AIM_KI * sum_yaw +
    //                YAW_AIM_KD * (yaw_rotation - last_yaw);
    // pitch_rotation = PITCH_AIM_KP * pitch_rotation + PITCH_AIM_KI * sum_pitch +
    //                  PITCH_AIM_KD * (pitch_rotation - last_pitch);

    // last_yaw = tmp_yaw;
    // last_pitch = tmp_pitch;

    // if (change_target) {
    //     sendTarget(serial, yaw_rotation, pitch_rotation, 3, 0);//表示目标切换
    // } else if (is_guessing) {
    //     sendTarget(serial, yaw_rotation, pitch_rotation, 4, 0);//表示猜测模式
    // } else {
    //     sendTarget(serial, yaw_rotation, pitch_rotation, shoot, 0);//跟随或发弹
    // }
    double x=d.x-320;
    double y=d.y-240;
    
    // double yaw = atan(x / FOCUS_PIXAL) * 180 / PI;
    // double pitch = atan(y / FOCUS_PIXAL) * 180 / PI;
    double yaw = atan(x / FOCUS_PIXAL);
    double pitch = atan(y / FOCUS_PIXAL);
    yaw-=word_yaw;
/**********************发送击打点中心坐标************************/
if (change_target) {
        sendTarget(serial, yaw, -pitch, 3, 0);//表示目标切换
    } else if (is_guessing) {
        sendTarget(serial,yaw, -pitch, 4, 0);//表示猜测模式
    } else {
        sendTarget(serial, yaw, -pitch, shoot, 0);//跟随或发弹
    }
}
/*********************************预测点坐标*************/
// if (change_target) {
//         sendTarget(serial, predict_point.x, -(predict_point.y), 3, 0);//表示目标切换
//     } else if (is_guessing) {
//         sendTarget(serial, predict_point.x, -(predict_point.y), 4, 0);//表示猜测模式
//     } else {
//         sendTarget(serial,predict_point.x, -(predict_point.y), shoot, 0);//跟随或发弹
//     }
// }



//----------------------------------------------------------------------------------------------------------------------
// 此函数用于发送数据给主控板
// ---------------------------------------------------------------------------------------------------------------------
void Energy::sendTarget(Serial &serial, float x, float y, float z, uint16_t u) {
    short x_tmp, y_tmp, z_tmp;
    uint8_t buff[10];

#ifdef WITH_COUNT_FPS
    static auto last_time = time(nullptr);
    static int fps = 0;
    time_t t = time(nullptr);
    if (last_time != t) {
        last_time = t;
        cout << "Energy: fps:" << fps << ", (" << x << "," << y << "," << z << "," << u << ")" << endl;
        curr_fps = fps;
        fps = 0;
    }
    fps += 1;
#endif

    if(isnan( x ) || isnan( y )){
        std::cout<<"data error: "<<x<<" : "<<y<<std::endl;
        return ;
    }
    buff[0] = 's';
    // printf("x:%f\n",x);
    float test = x;
    memcpy(buff + 1, &test, 4);
    test = y;
    memcpy(buff + 5, &test, 4);
    // printf("data: %f\n", *(float*)(buff + 1));
    buff[9] = 'e';

    // x_tmp = static_cast<short>(x * (32768 - 1) / 100);
    // y_tmp = static_cast<short>(y * (32768 - 1) / 100);
    // z_tmp = static_cast<short>(z * (32768 - 1) / 100);
    // buff[0] = 's';
    // buff[1] = static_cast<char>((x_tmp >> 8) & 0xFF);
    // buff[2] = static_cast<char>((x_tmp >> 0) & 0xFF);
    // buff[3] = static_cast<char>((y_tmp >> 8) & 0xFF);
    // buff[4] = static_cast<char>((y_tmp >> 0) & 0xFF);
    // buff[5] = static_cast<char>((z_tmp >> 8) & 0xFF);
    // buff[6] = static_cast<char>((z_tmp >> 0) & 0xFF);
    // buff[7] = 'e';

    // buff[7] = static_cast<char>((u >> 8) & 0xFF);
    // buff[8] = static_cast<char>((u >> 0) & 0xFF);
    // buff[9] = 'e';
    serial.WriteData(buff, sizeof(buff));
    send_cnt += 1;
//    LOGM(STR_CTR(WORD_LIGHT_PURPLE, "send"));
}
