//
// Created by xinyang on 19-7-11.
//

#include <armor_finder/armor_finder.h>
#include <config/setconfig.h>
#include <log.h>
#include <iostream>

static bool sendTarget(Serial &serial, int16_t x, int16_t y , double z/*, uint16_t shoot_delay*/) {
    static short x_tmp, y_tmp, z_tmp;
    uint8_t buff[8];

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

    *((uint16_t*)(buff + 1)) = x;
    *((uint16_t*)(buff + 3)) = y;
    *((uint16_t*)(buff + 5)) = z;



    // buff[5] = static_cast<char>((z_tmp >> 8) & 0xFF);
    // buff[6] = static_cast<char>((z_tmp >> 0) & 0xFF);
    // buff[7] = static_cast<char>((shoot_delay >> 8) & 0xFF);
    // buff[8] = static_cast<char>((shoot_delay >> 0) & 0xFF);
    buff[7] = 'e';
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
    if (target_box.rect == cv::Rect2d()) return false;
    if (shoot_delay) {
        LOGM(STR_CTR(WORD_BLUE, "next box %dms"), shoot_delay);
    }
    cv::Rect2d rect;
    if(is_kalman) rect=kal_rect;
    else rect = target_box.rect;
    int16_t dx = rect.x + rect.width / 2 - IMAGE_CENTER_X;
    int16_t dy = -(rect.y + rect.height / 2 - IMAGE_CENTER_Y);
    // double yaw = atan(dx / FOCUS_PIXAL) * 180 / PI;
    // double pitch = atan(dy / FOCUS_PIXAL) * 180 / PI;
    // dy+=20;
    dy+=15;
    double dist = DISTANCE_HEIGHT / rect.height;
    return sendTarget(serial, dx, dy, dist/*, shoot_delay*/);
}
