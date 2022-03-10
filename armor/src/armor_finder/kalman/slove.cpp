#include<kalman/slove.h>

void kal_test::Init(int x,int y) { kalman.Init(x,y); }

kal_test::kal_test(){
    _Kalman::Matrix_xxd A = _Kalman::Matrix_xxd::Identity(); //单位矩阵
    _Kalman::Matrix_zxd H{1,0};
    // H(0, 0) = 1; //[1,0]
    _Kalman::Matrix_xxd R;
    R(0, 0) = 0.01; //[0.01,0]  //调参 越低越相信估计值
    for (int i = 1; i < S; i++) {
        R(i, i) = 100;
    }
    _Kalman::Matrix_zzd Q{0.2}; //调参  越低越相信测量值
    _Kalman::Matrix_x1d init{0, 0};
    kalman = _Kalman(A, H, R, Q, init, 0);
}

int kal_test::slove(int x,int t){
    _Kalman::Matrix_z1d temp{x};
    _Kalman::Matrix_x1d temp2;
    // std::cout<<"t:"<<std::endl;
    // std::cout<<t<<std::endl<<std::endl;
    temp2=kalman.update(temp,0);
    int ans=temp2(0,0);
    return ans;
}