#include<kalman/slove.h>

void kal_test::Init(int x,double t) { last_x=x; kalman.Init(x,t); }

kal_test::kal_test(){
    _Kalman::Matrix_xxd A = _Kalman::Matrix_xxd::Identity(); //单位矩阵
    _Kalman::Matrix_zxd H{1,0};
    _Kalman::Matrix_xxd R;
    R(0, 0) = 1;
    for (int i = 1; i < S; i++) {
        R(i, i) = 0.01;
    }
    _Kalman::Matrix_zzd Q{100};
    _Kalman::Matrix_x1d init{0, 0};
    kalman = _Kalman(A, H, R, Q, init, 0);
}

int kal_test::slove(int x,double t){
    int dx=x-last_x;
    last_x=x;
    x+=2*dx;
    return x;
    _Kalman::Matrix_z1d temp{x};
    _Kalman::Matrix_x1d temp2;
    // std::cout<<"t:";
    // std::cout<<t<<std::endl<<std::endl;
    temp2=kalman.update(temp,t);
    int ans=temp2(0,0);
    return ans;
}