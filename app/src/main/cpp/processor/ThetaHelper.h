//
// Created by 张哲华 on 2019/3/1.
//

#ifndef GRYOSTABLE_THETAHELPER_H
#define GRYOSTABLE_THETAHELPER_H

#include "opencv2/opencv.hpp"
#include "ThreadContext.h"
#include "Filter.h"
#include "Quaternion.h"
#include <queue>
using namespace cv;
using namespace std;

class ThetaHelper {
private:
//    Mat inmat=(cv::Mat_<double>(3, 3)<<1440.0,0.0,540.0, 0.0,1440.0,960.0,0.0,0.0,1.0);
//    Mat inmat=(cv::Mat_<double>(3, 3)<<600.0,0.0,616.0, 0.0,600.0,956.0,0.0,0.0,1.0);
    Mat inmat=(cv::Mat_<double>(3, 3)<<1430.2,0.0,505.7, 0.0,1422.9,922.1,0.0,0.0,1.0);//OnePlus 6T
//    Mat inmat=(cv::Mat_<double>(3, 3)<<1492.89950430177,0.0,940.850079740057, 0.0,1496.13805384036,552.228021875255,0.0,0.0,1.0);//demo board
    Mat vertex=(cv::Mat_<double>(3, 4)<<0.0,0.0,1080.0,1080.0,0.0,1920.0,1920.0,0.0,1.0,1.0,1.0,1.0);
    Mat cropvertex=(cv::Mat_<double>(3, 4)<<108.0,108.0,972.0, 972.0, 192.0 , 1728.0, 1728.0, 192.0,1.0,1.0,1.0,1.0);
    Mat I=(cv::Mat_<double>(3, 3)<<1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0);


    int c;
    int gyindex;
    int findex;
    int angledex;

    vector<double> roxl;
    vector<double> royl;
    vector<double> rozl;
    vector<double> Timeg;
    vector<double> Timeframe;
    vector<double> oldx;
    vector<double> oldy;
    vector<double> oldz;
    double lastx;
    double lasty;
    double lastz;


    cv::Mat result ;
    cv::Mat RR;

    double px;
    double py;
    double pz;
    double q;
    double rx;
    double ry;
    double rz;
    double kx;
    double ky;
    double kz;
    cv::Vec<double, 3> lasttheta;
    cv::Vec<double, 3> lastt;
    void cropControl(Mat& RR);
    bool isInside(cv::Mat cropvertex, cv::Mat newvertex);
    int gyro_count_ = 0;
    float avg_gyro_[3];

public:
    void init();
    Mat getRR(Mat oldRotation, Mat newRotation);
    std::vector<cv::Vec<double, 3>> getTheta();
    cv::Vec<double, 3> getNewTheta(cv::Vec<double, 3> oldtheta);
    cv::Mat getRotationMat(cv::Vec<double, 3> theta);
    void getR(double timestamp, Mat *matR, bool isCrop);
    void putValue(double timestamp, float x, float y, float z);

private:
    int rs_frame_index_;
    int rs_gyro_index_;
    cv::Vec<double, 4> rs_last_theta_;
    double rs_last_x_, rs_last_y_, rs_last_z_;
    std::vector<cv::Vec<double, 4>> rs_gyro_theta_;
    const bool is_use_drift_ = false;
    float x_drift_ = 0;
    float y_drift_ = 0;
    float z_drift_ = 0;
    Filter filter_;
    std::queue<cv::Mat> old_rotation_queue_;
    Quaternion last_q_ = Quaternion::EulerToQuaternion(0, 0, 0);

    bool is_first_push_ = true;
    void WriteToFile(FILE* file, float x, float y, float z, int gyro_count_);
public:
    std::vector<cv::Vec<double, 4>> GetRsTheta();
    void RsChangeVectorToMat(cv::Mat* rs_out_Mat);
////********************卡尔曼滤波处理陀螺仪数据*****************************////
private:
    float noise_[3] = {0};
    float xw_[5] = {0};
    float yw_[5] = {0};
    float zw_[5] = {0};
    int count_ = 0;
    float p_[3] = {0};
    float xa_[3] = {0};
private:
    void GetNoise();
    void GetGyro(float x, float y, float z);

};


#endif //GRYOSTABLE_THETAHELPER_H
