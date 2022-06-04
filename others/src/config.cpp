#include <config.h>

double A03;
double A06;
double A10;
double A13;
double A16;
double A20;
double A23;
double A26;
double A30;
double A33;
double A36;
double A40;
double A43;
double A46;
double A50;

void readAconfig(std::string path){
        cv::FileStorage fin(path, cv::FileStorage::READ);

        fin["A03"] >> A03;
        fin["A06"] >> A06;
        fin["A10"] >> A10;
        fin["A13"] >> A13;
        fin["A16"] >> A16;
        fin["A20"] >> A20;
        fin["A23"] >> A23;
        fin["A26"] >> A26;
        fin["A30"] >> A30;
        fin["A33"] >> A33;
        fin["A36"] >> A36;
        fin["A40"] >> A40;
        fin["A43"] >> A43;
        fin["A46"] >> A46;
        fin["A50"] >> A50;
}