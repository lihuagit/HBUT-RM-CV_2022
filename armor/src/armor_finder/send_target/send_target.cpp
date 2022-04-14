//
// Created by xinyang on 19-7-11.
//

#include <armor_finder/armor_finder.h>
#include <config/setconfig.h>
#include <log.h>
#include <iostream>
#include <predictor/PredictorKalman.h>
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

bool ArmorFinder::sendBoxPosition(uint16_t shoot_delay) {
    static kal_test kal_dis;
    static kal_test kal_yaw;
    static double last_yaw=10;
    if (target_box.rect == cv::Rect2d()) return false;
    if (shoot_delay) {
        LOGM(STR_CTR(WORD_BLUE, "next box %dms"), shoot_delay);
    }
    cv::Rect2d rect;
    if(is_kalman) rect=kal_rect;
    else rect = target_box.rect;
    rect.y=target_box.rect.y;
    int16_t dx = rect.x + rect.width / 2 - IMAGE_CENTER_X;
    int16_t dy = -(rect.y + rect.height / 2 - IMAGE_CENTER_Y);
    // double yaw = atan(1.0*dx / FOCUS_PIXAL) / 180 * PI;
    // double pitch = atan(1.0*dy / FOCUS_PIXAL) / 180 * PI;
    float yaw = atan(1.0*dx / FOCUS_PIXAL);
    double pitch = atan(1.0*dy / FOCUS_PIXAL);
    double dist = DISTANCE_HEIGHT / rect.height*0.666666;
    double now_t;
    getsystime(now_t);
    printf("dx:%d\n",dx);
    printf("now_t:%lf\n",now_t);
    // std::cout<<yaw<<" "<<pitch<<std::endl;
    printf("d___yaw:%f\n",yaw);
    // yaw+=word_yaw;
    float res_yaw=yaw-word_yaw;
    // if(fabs(res_yaw)>3.1415926) res_yaw=last_yaw;
    //  if(fabs(yaw-last_yaw)<=0.007)
        //  res_yaw=kal_yaw.slove(res_yaw,now_t);
    //  else
        //  kal_yaw.Init(yaw,now_t);
     last_yaw=res_yaw;
    // std::cout<<"word_yaw:"<<word_yaw<<std::endl;
    printf("res_yaw:%f\n",res_yaw);
    printf("word_yaw:%f\n",word_yaw);
    // std::cout<<yaw<<" "<<pitch<<std::endl;
    // yaw-=word_yaw;
    dist=kal_dis.slove(dist,0);
    dy-=50;
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
    // getsystime(kal_t);
    // kal_x.slove(dx,kal_t);
    // std::cout<<yaw<<" "<<pitch<<std::endl;
    // return sendTarget(serial, dx, dy, dist/*, shoot_delay*/);
    return sendTarget(serial, res_yaw, dy, dist/*, shoot_delay*/);
}
