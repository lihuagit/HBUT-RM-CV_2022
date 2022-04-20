/**
 * @file slove.h
 * @brief 
 * kalman用来简介调用kalman类 实现卡尔曼滤波 只有一个外部接口slove(int x,int t)
 * 调参在kal_test的构造函数里
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-01-17
 * 
 */

#ifndef _SLOVE_H_
#define _SLOVE_H_

#include <predictor/kalman.h>
#include <iostream>
#include <options.h>

constexpr int S=2;
class predictorKalman{
    using _Kalman = Kalman<1, S>;
    _Kalman kalman;
public:
    predictorKalman();
    std::vector<double> predictor(double x,double t);
    void Init(double x,double t);
};

#endif