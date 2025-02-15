/**
 * @file armor_finder.cpp
 * @brief 自瞄主函数实现
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-05-16
 * 
 */

/*===========================================================================*/
/*                               使用本代码的兵种                               */
/*===========================================================================*/
/*   _______________   _______________   _______________   _______________   */
/*  |     _____     | |     _  _      | |     ____      | |     _____     |  */
/*  ||   |___ /    || ||   | || |    || ||   | ___|    || ||   |___  |   ||  */
/*  ||     |_ \    || ||   | || |_   || ||   |___ \    || ||      / /    ||  */
/*  ||    ___) |   || ||   |__   _|  || ||    ___) |   || ||     / /     ||  */
/*  ||   |____/    || ||      |_|    || ||   |____/    || ||    /_/      ||  */
/*  |_______________| |_______________| |_______________| |_______________|  */
/*                                                                           */
/*===========================================================================*/

#define LOG_LEVEL LOG_NONE
#include <log.h>
#include <show_images/show_images.h>
#include <opencv2/highgui.hpp>
#include <armor_finder/armor_finder.h>
#include <ctime>
#include <chrono>

std::map<int, string> id2name = {                               //装甲板id到名称的map
        {-1, "OO"},{ 0, "NO"},
        { 1, "B1"},{ 2, "B2"},{ 3, "B3"},{ 4, "B4"},{ 5, "B5"},{ 6, "B6"},{ 7, "B7"},{ 8, "B8"},
        { 9, "R1"},{10, "R2"},{11, "R3"},{12, "R4"},{13, "R5"},{14, "R6"},{15, "R7"},{16, "R8"}
};

std::map<string, int> name2id = {                               //装甲板名称到id的map
        {"OO", -1},{"NO",  0},
        {"B1",  1},{"B2",  2},{"B3",  3},{"B4",  4},{"B5",  5},{"B6",  6},{"B7",  7},{"B8",  8},
        {"R1",  9},{"R2",  10},{"R3", 11},{"R4", 12},{"R5", 13},{"R6", 14},{"R7", 15},{"R8", 16},
};

std::map<string, int> prior_blue = {
        {"B8", 0}, {"B1", 1}, {"B3", 2}, {"B4", 2}, {"B5", 2}, {"B7", 3}, {"B2", 4},{"B6", 4},
        {"R8", 5}, {"R1", 6}, {"R3", 7}, {"R4", 7}, {"R5", 7}, {"R7", 8}, {"R2", 9},{"R6", 9},
        {"NO", 10},
};

std::map<string, int> prior_red = {
        {"B8", 0}, {"B1", 1}, {"B3", 2}, {"B4", 2}, {"B5", 2}, {"B7", 3}, {"B2", 4},{"B6", 4},
        {"R8", 5}, {"R1", 6}, {"R3", 7}, {"R4", 7}, {"R5", 7}, {"R7", 8}, {"R2", 9},{"R6", 9},
        {"NO", 10},
};

ArmorFinder::ArmorFinder(uint8_t &color, Serial &u, const string &paras_folder, const uint8_t &anti_top) :
        serial(u),
        enemy_color(color),
        is_anti_top(anti_top),
        state(STANDBY_STATE),
        anti_switch_cnt(0),
        classifier(paras_folder),
        contour_area(0),
        tracking_cnt(0) {
        sendData.mode = 'N';
}

void ArmorFinder::run(cv::Mat &src) {
    static int cnt_useless = -1;
    static int fps = 0, fps_count = 0;
    static auto t1 = std::chrono::system_clock::now();
    static int last_cnt=0;

    im2show=src.clone();
    getsystime(frame_time); //　获取当前帧时间(不是足够精确)
//    stateSearchingTarget(src);                    // for debug
//    goto end;
    switch (state) {
        case SEARCHING_STATE:
            if (stateSearchingTarget(src)) {
                if ((target_box.rect & cv::Rect2d(0, 0, 640, 480)) == target_box.rect) { // 判断装甲板区域是否脱离图像区域
                    if (!classifier) {                                          /* 如果分类器不可用 */
                        cv::Mat roi = src(target_box.rect).clone(), roi_gray;  /* 就使用装甲区域亮点数判断是否跟丢 */
                        cv::cvtColor(roi, roi_gray, cv::COLOR_RGB2GRAY);
                        cv::threshold(roi_gray, roi_gray, 180, 255, cv::THRESH_BINARY);
                        contour_area = cv::countNonZero(roi_gray);
                    }
                    tracker = TrackerToUse::create();                       // 成功搜寻到装甲板，创建tracker对象
                    tracker->init(src, target_box.rect);
                    if(is_anti_top) sendData.mode='Y';
                    state = TRACKING_STATE;
                    tracking_cnt = 0;
                    LOGM(STR_CTR(WORD_LIGHT_CYAN, "into track"));
                }
            }
            break;
        case TRACKING_STATE:
            if (!stateTrackingTarget(src) || ++tracking_cnt > 100) {    // 最多追踪100帧图像
                state = SEARCHING_STATE;
                LOGM(STR_CTR(WORD_LIGHT_YELLOW, "into search!"));
            }
            break;
        case STANDBY_STATE:
            stateStandBy(src); // currently meaningless
            break;
        default:
            break;
    }
    
    if(state != STANDBY_STATE){
        if(target_box.rect != cv::Rect2d()) {
            anti_top_cnt = 0;
            sendBoxPosition(0);
            last_box = target_box;
            double cen_x=target_box.rect.x + target_box.rect.width/2;
            if(cen_x > src.cols/2 && is_anti_top)
                state=SEARCHING_STATE;
            sendData.mode='N';
        }
    }
    else  {
        // 跟丢 || 切换装甲板时 保留缓冲时间 一定程度减少程序不稳定频繁切换装甲板
        if(last_box.id != target_box.id) last_cnt++;
        else last_cnt=0;
        if(target_box.rect != cv::Rect2d()){
            if( last_cnt >= 5 ) last_box = target_box, last_cnt = 0;
            sendBoxPosition(0);
        }
    }

    if (show_armor_box) {                 // 根据条件显示当前目标装甲板
        // if(is_kalman)
            // showArmorBox("box", src, target_box,kal_rect);
        // else showArmorBox("box", src, target_box);
        showArmorBox("box", src, target_box);
        
        fps_count++;
        auto t2 = std::chrono::system_clock::now();
        if (duration_cast<std::chrono::milliseconds>(t2 - t1).count() >= 1000) {
            fps = fps_count;
            fps_count = 0;
            t1 = t2;
        }
        char TextFPS[20];
        sprintf(TextFPS,"fps={ %d }\0",fps);
        cv::putText(im2show, TextFPS, {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0));

        cv::imshow("kalman",im2show);
        cv::waitKey(1);
    }
}

/**
 * @brief 
 * kalman更新
 * 已弃用
 */
void ArmorFinder::kal_run(){
    cv::Point2f box_center=target_box.getCenter();
    double newx,newy;
    getsystime(now_t);
    // newx=kal_yaw.predictor(box_center.x,now_t);
    newy=box_center.y;
    int w=target_box.rect.width;
    int h=target_box.rect.height;
    kal_rect=cv::Rect2f(newx-(w/2),newy-(h/2),w,h);
}
