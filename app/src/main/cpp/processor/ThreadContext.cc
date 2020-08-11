//
// Created by 张哲华 on 20/09/2017.
//

#include "ThreadContext.h"
using namespace threads;

MySemaphore* ThreadContext::klt_semaphore = nullptr;
MySemaphore* ThreadContext::mc_semaphore = nullptr;
MySemaphore* ThreadContext::out_semaphore = nullptr;
MySemaphore* ThreadContext::read_semaphore= nullptr;
MySemaphore* ThreadContext::rs_semaphore_ = nullptr;

vector<Point2i> ThreadContext::kltList;
list<list<vector<Point2f>>> ThreadContext::trj;
queue<Vec<double, 3>> ThreadContext::rTheta;
Mat ThreadContext::frameVec[ThreadContext::BUFFERSIZE];
Mat ThreadContext::stableTransformVec[ThreadContext::BUFFERSIZE];
Mat ThreadContext::stableRVec[ThreadContext::BUFFERSIZE];

Mat ThreadContext::rs_out_theta_[ThreadContext::BUFFERSIZE];
Mat ThreadContext::rs_Mat_[ThreadContext::BUFFERSIZE][ThreadContext::KRsStripNum_];
double ThreadContext::guass_weight_[11];
std::queue<std::vector<cv::Point2f>> ThreadContext::feature_by_r_;
////**************测试****************////
std::queue<cv::Mat> ThreadContext::r_convert_que;
std::queue<cv::Mat> ThreadContext::r_convert_que1;
cv::Mat ThreadContext::last_old_Rotation_ = cv::Mat::eye(3, 3, CV_64F);
std::queue<double> ThreadContext::gyro_z_theta_que;
void ThreadContext::Init() {
    Release();
    klt_semaphore = new MySemaphore(0);
    mc_semaphore = new MySemaphore(0);
    out_semaphore = new MySemaphore(0);
    read_semaphore = new MySemaphore(BUFFERSIZE);
    rs_semaphore_ = new MySemaphore(0);
    CreateGaussWeight(guass_weight_);
}

void ThreadContext::Release() {
    if (read_semaphore != nullptr) {
        delete read_semaphore;
        read_semaphore = nullptr;
    }
    if (klt_semaphore != nullptr) {
        delete klt_semaphore;
        klt_semaphore = nullptr;
    }
    if (mc_semaphore != nullptr) {
        delete mc_semaphore;
        mc_semaphore = nullptr;
    }
    if (out_semaphore != nullptr) {
        delete out_semaphore;
        out_semaphore = nullptr;
    }
    kltList.clear();
}

void ThreadContext::CreateGaussWeight(double *gaussWeight) {
    int num = 0;
    for(int i = -5; i < 6; i++){
        double temp = -(i*i)/2.0;
        double xishu = 1.0/sqrt(2.0*pi);
        *(gaussWeight+num) = xishu*exp(temp);
        num++;
    }
}