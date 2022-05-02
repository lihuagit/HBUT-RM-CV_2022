#ifndef _SEND_DATA_H_
#define _SEND_DATA_H_

struct send_data
{
    /* data */
    // 发送数据包
    float send_yaw;
    float send_pitch;
    float send_dist;
};

struct src_date
{
    /* data */
    // 原始数据
    cv::Point2f p[4];
    double rec_yaw;
    double id;
    double now_t;
};

#endif      // _SEND_DATA_H_