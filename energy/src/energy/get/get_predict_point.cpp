//
// Created by xixiliadorabarry on 1/24/19.
//

#include "energy/energy.h"
#include "energy/constant.h"

using namespace cv;
using std::cout;
using std::endl;
using std::vector;

//计算实时角速度
void Energy::calculateRotateSpeed() {
//定义静态过去和现在角度；
    _circleAngle180 = target_polar_angle;
    //旋转角度处理
    if (circle_center_point.y < target_point.y) {
        _circleAngle360 = 360.0f - _circleAngle180;
        _circleAngle180 = -_circleAngle180;
    } else {
        _circleAngle360 = _circleAngle180;
    }
    static double nowAngle = 0.0f;
    static double lastAngle = 0.0f;
    static int count = 0;
    //定义过去和现在时间ms
    static double lastTime = (double) cv::getTickCount() / cv::getTickFrequency() * 1000; //过去，1000 *总次数/一秒内重复的次数= 时间(ms)
    double curTime = (double) cv::getTickCount() / cv::getTickFrequency()* 1000;//现在
    //cout<<curTime<<"ms"<<endl;
    //如果叶片没有跳变，则把过去和现在角度以及过去和现在速度置零
    
    if (change_target==false) {
        lastAngle = nowAngle = _predictbig.lastRotateSpeed =_predictbig.nowRotateSpeed = 0.0f;
        return;
    }
    //如果过去角度已经被清零，则过去角度进行初始化为现在绝对角度
    if (lastAngle == 0.0f) {
        lastAngle = _circleAngle360;
        return;
    }
    //每0.1s一次数据刷新
    if (curTime - lastTime < 100) {
        return;
    }
    //帧数递增
    count++;
    nowAngle = _circleAngle360;
    //计算实时角速度,rad/s
    _predictbig.nowRotateSpeed = (float) fabs(((nowAngle - lastAngle)*PI/180) * (1000.0f / (curTime - lastTime)));
    //过去角度和时间更新
    lastAngle = nowAngle;
    lastTime = curTime;
    //如果过去角速度已被清零，则对过去速度进行更新
    if (_predictbig.lastRotateSpeed == 0.0f) {
        _predictbig.lastRotateSpeed =_predictbig.nowRotateSpeed;
        return;
    }
    //防止出现异常数据，如果现在角速度绝对值大于5
    if (_predictbig.nowRotateSpeed > 5 ||_predictbig.nowRotateSpeed < -5) {
        return;
    }
    //如果速度没有替换最小速度，则计数加1
    if (_predictbig.nowMinSpeed > _predictbig.nowRotateSpeed) {
        _predictbig.nowMinSpeed = _predictbig.nowRotateSpeed;
    } else {
        _predictbig.minSameNumber++;
    }
    //如果速度没有替换最大速度，则计数加1
    if (_predictbig.nowMaxSpeed < _predictbig.nowRotateSpeed) {
        _predictbig.nowMaxSpeed = _predictbig.nowRotateSpeed;

    } else {
        _predictbig.maxSameNumber++;
    }
    //如果连续20帧没有刷新最小速度，则该速度为波谷速度（该速度一旦更新，便不再更新）
    if (_predictbig.minSameNumber > 20 && !_predictbig.minSpeedFlag) {
        _predictbig.realMinSpeed = _predictbig.nowMinSpeed;
        _predictbig.minSpeedFlag = true;
        
    }
    //如果连续20帧没有刷新最大速度，则该速度为波峰速度（该速度一旦更新，便不再更新）
    if (_predictbig.maxSameNumber > 20 && !_predictbig.maxSpeedFlag) {
        _predictbig.realMaxSpeed = _predictbig.nowMaxSpeed;
        _predictbig.maxSpeedFlag = true;
        
    }

    _predictbig.realRotateSpeed = _predictbig.nowRotateSpeed;
    //cout<<_predictbig.realRotateSpeed<<endl;
    _predictbig.speedType = (_predictbig.nowRotateSpeed > _predictbig.lastRotateSpeed ? SPEED_UP : SPEED_DOWN);
    
}
float Energy::calculateShootTime() {
    
    if (_predictbig.realRotateSpeed <= 0.0f) {
        return float();
    }
    //spd=0.785*sin(1.884*t)+1.305
    float possibleTime[2];
    float realTime;
    //速度超限要改
    if (_predictbig.realRotateSpeed < _predictbig.para - _predictbig.amplitude) {
        _predictbig.realRotateSpeed = _predictbig.para - _predictbig.amplitude;
    }
    if (_predictbig.realRotateSpeed > _predictbig.para + _predictbig.amplitude) {
        _predictbig.realRotateSpeed = _predictbig.para + _predictbig.amplitude;
    }
    possibleTime[0] = (asinf((_predictbig.realRotateSpeed - _predictbig.para) / _predictbig.amplitude)) / _predictbig.rotateIndex;
    possibleTime[1] = (possibleTime[0] > 0 ? PI_F() / (_predictbig.rotateIndex) - possibleTime[0] : PI_F() / (-_predictbig.rotateIndex) - possibleTime[0]);
    cout << "    " << _predictbig.speedType << endl;
    realTime = (_predictbig.speedType == SPEED_UP ? possibleTime[0] : possibleTime[1]);
    //cout << "time--" << fabs(possibleTime[0] - possibleTime[1]) << endl;
    return realTime;
}

//----------------------------------------------------------------------------------------------------------------------
// 此函数获取预测点坐标
// ---------------------------------------------------------------------------------------------------------------------
void Energy::getPredictPoint(cv::Point target_point) {
    float time = 0.0f;
     //计算实时角速度
    calculateRotateSpeed();
    //计算积分开始时刻（该时刻作为角速度函数的横坐标）
    time = calculateShootTime();
    //cout<<"time = 0.0f"<<endl;
    // if (time == 0.0f) {
    //     return float();
    // }
    if (is_small) {
        if (energy_rotation_direction == 1) predict_rad = predict_rad_norm;
        else if (energy_rotation_direction == -1) predict_rad = -predict_rad_norm;
        rotate(target_point);
    } 
    else if (is_big) {//predict_point = target_point;
        _delayTime = DELAY_TIME+0.25f;//射速大于22时注释，小于22时加0.25
        _realAddAngle = (_predictbig.amplitude / _predictbig.rotateIndex
                         * (cosf(_predictbig.rotateIndex * time) - cosf(_predictbig.rotateIndex * (time + _delayTime))) + _predictbig.para * DELAY_TIME)
                        * 180 / PI_F() + _para;
        //return _realAddAngle;
        if (energy_rotation_direction == 1) 
        {
            _predictbig.para += FRONT_BACK_SIN;
            _predictbig.amplitude += FRONT_BACK_SIN;
            predict_rad = -_realAddAngle;
        }
        else if (energy_rotation_direction == -1) 
        {
            _predictbig.para -= FRONT_BACK_SIN;
            _predictbig.amplitude -= FRONT_BACK_SIN;
            predict_rad = _realAddAngle;
        }
        rotate(target_point);
    }
}

