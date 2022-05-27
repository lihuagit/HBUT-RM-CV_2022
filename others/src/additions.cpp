//
// Created by sjturm on 19-5-17.
//

#include <cstring>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio_c.h>
#include <additions.h>
#include <camera/camera_wrapper.h>
#include <energy/energy.h>
#include <armor_finder/armor_finder.h>
#include <log.h>

#define RECEIVE_LOG_LEVEL LOG_MSG

using namespace std;
using namespace cv;

extern WrapperHead *video;

extern Serial serial;
extern uint8_t last_state;

extern ArmorFinder armor_finder;
extern Energy energy;

extern McuData mcu_data;

void uartReceive(Serial *pSerial) {
    char buffer[40];
    LOGM(STR_CTR(WORD_LIGHT_WHITE, "data receive start!"));
    while (true) {
        memset(buffer, 0, sizeof(buffer));
        pSerial->ReadData((uint8_t *) buffer, 15);
       //std::cout<<strlen(buffer)<<std::endl;
        if(strlen(buffer)>=5){
            char mode=buffer[0];
            char mode_shoot=buffer[1];
            memcpy(&(armor_finder.word_yaw), buffer+2, 4);
            memcpy(&(armor_finder.word_pitch), buffer+6, 4);
            // sscanf(buffer+1,"%f",&(armor_finder.word_yaw));
            // std::cout<<"armor_finder.word_yaw: ";
            // std::cout<<armor_finder.word_yaw<<std::endl;
            // std::cout<<"armor_finder.word_pitch: ";
            // std::cout<<armor_finder.word_pitch<<std::endl;
            //  energy.word_yaw=armor_finder.word_yaw;
            // energy.word_pitch=armor_finder.word_pitch;
            //std::cout<<"mode_1 is::  "<<mode_1<<std::endl;
            // 预测模式
            if(mode == 'Y'){
                mcu_data.state=ARMOR_STATE;
                // shoot_delay_t=0.12;  // 射击延迟
                // shoot_v=15;         // 单速
            // 不预测
            }else if(mode == 'B'){
                mcu_data.state=ARMOR_STATE;
                // shoot_delay_t=0;  // 射击延迟
                // shoot_v=0;         // 单速
            // 小符
            }else if(mode == 'D'){
                mcu_data.state=SMALL_ENERGY_STATE;
                // shoot_delay_t=0;  // 射击延迟
                // shoot_v=0;         // 单速
            // 大符
            }else if(mode == 'X'){
                mcu_data.state=BIG_ENERGY_STATE;
                // shoot_delay_t=0;  // 射击延迟
                // shoot_v=0;         // 单速
            // 切换颜色
            }else if(mode == 'Q'){
                if(mcu_data.enemy_color == ENEMY_RED)
                    mcu_data.enemy_color=ENEMY_BLUE;
                else mcu_data.enemy_color=ENEMY_RED;
            }
            if(mode_shoot==1) shoot_v=14;
            else if(mode_shoot==2) shoot_v=17;
            else if(mode_shoot==2) shoot_v=28;
        }
    }
}

cv::VideoWriter initVideoWriter(const std::string &filename_prefix) {
    cv::VideoWriter video;
    std::ifstream in(filename_prefix + "cnt.txt");
    int cnt = 0;
    if (in.is_open()) {
        in >> cnt;
        in.close();
    }
    std::string file_name = filename_prefix + std::to_string(cnt) + ".avi";
    cnt++;
    std::ofstream out(filename_prefix + "cnt.txt");
    if (out.is_open()) {
        out << cnt << std::endl;
        out.close();
    }
    video.open(file_name, CV_FOURCC('P', 'I', 'M', '1'), 90, cv::Size(640, 480), true);
    return video;
}

bool checkReconnect(bool is_camera_connect) {
    if (!is_camera_connect) {
        int curr_gain = ((CameraWrapper* )video)->gain;
        int curr_exposure = ((CameraWrapper* )video)->exposure;
        delete video;
        video = new CameraWrapper(curr_exposure, curr_gain, 0/*, "armor"*/);
        is_camera_connect = video->init();
    }
    return is_camera_connect;
}

auto video_writer = initVideoWriter(PROJECT_DIR"/video/");

void saveVideos(const cv::Mat &gimbal_src) {
    if (!gimbal_src.empty()) {
        video_writer.write(gimbal_src);
    } else return;
}

void showOrigin(const cv::Mat &src) {
    if (!src.empty()) {
        imshow("origin", src);
        cv::waitKey(1);
    } else return;
}

void extract(cv::Mat &src) {//图像预处理，将视频切成640×480的大小
    if (src.empty()) return;
    float length = static_cast<float>(src.cols);
    float width = static_cast<float>(src.rows);
    if (length / width > 640.0 / 480.0) {
        length *= 480.0 / width;
        resize(src, src, cv::Size(length, 480));
        src = src(Rect((length - 640) / 2, 0, 640, 480));
    } else {
        width *= 640.0 / length;
        resize(src, src, cv::Size(640, width));
        src = src(Rect(0, (width - 480) / 2, 640, 480));
    }
}

double getPointLength(const cv::Point2f &p) {
    return sqrt(p.x * p.x + p.y * p.y);
}
