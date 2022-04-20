#include <predictor/PredictorKalman.h>
#include <math.h>

void predictorKalman::Init(double x,double t) { kalman.reset(x,t); }

predictorKalman::predictorKalman(){
    _Kalman::Matrix_xxd A = _Kalman::Matrix_xxd::Identity(); //单位矩阵
    _Kalman::Matrix_zxd H{1,0};
    _Kalman::Matrix_xxd R;
    R(0, 0) = 0.01;
    for (int i = 1; i < S; i++) {
        R(i, i) = 100;
        // R(i, i) = 0.01;  //for dis   
    }
    _Kalman::Matrix_zzd Q{4};
    _Kalman::Matrix_x1d init{0, 0};
    kalman = _Kalman(A, H, R, Q, init, 0);
}

std::vector<double> predictorKalman::predictor(double x,double t){
    _Kalman::Matrix_z1d Zk{x};
    _Kalman::Matrix_x1d ansXk;
    ansXk=kalman.update(Zk,t);
    std::vector<double> ans(5);
    ans.emplace_back(ansXk(0,0));
    ans.emplace_back(ansXk(1,0));
    return ans;
}
