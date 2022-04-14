#include <predictor/PredictorKalman.h>
#include <math.h>

void kal_test::Init(double x,double t) { last_x=x; kalman.reset(x,t); }

kal_test::kal_test(){
    _Kalman::Matrix_xxd A = _Kalman::Matrix_xxd::Identity(); //单位矩阵
    _Kalman::Matrix_zxd H{1,0};
    _Kalman::Matrix_xxd R;
    R(0, 0) = 0.01;
    for (int i = 1; i < S; i++) {
        // R(i, i) = 100;
        R(i, i) = 0.01;  //for dis
    }
    _Kalman::Matrix_zzd Q{100};
    _Kalman::Matrix_x1d init{0, 0};
    kalman = _Kalman(A, H, R, Q, init, 0);
}

double kal_test::slove(double x,double t){
    _Kalman::Matrix_z1d temp{x};
    _Kalman::Matrix_x1d temp2;
    temp2=kalman.update(temp,t);
    double ans=temp2(0,0)+shoot_delay_t*temp2(1,0);
    return ans;
}
