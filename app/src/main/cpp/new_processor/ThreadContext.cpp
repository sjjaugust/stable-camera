//
// Created by ShiJJ on 2020/4/13.
//

#include "ThreadContext.h"

MySemaphore* ThreadContext::read_semaphore_ = nullptr;
MySemaphore* ThreadContext::out_semaphore_ = nullptr;
MySemaphore* ThreadContext::cm_semaphore_ = nullptr;
MySemaphore* ThreadContext::rs_semaphore_ = nullptr;
const int ThreadContext::BUFFER_SIZE_ = 20;
cv::Mat ThreadContext::frame_vec_[ThreadContext::BUFFER_SIZE_];
cv::Mat ThreadContext::quaternion_vec_[ThreadContext::BUFFER_SIZE_];
cv::Mat ThreadContext::stable_vec_[ThreadContext::BUFFER_SIZE_];
cv::Mat ThreadContext::rs_convert_mat_;
const cv::Mat ThreadContext::RR2stableVec = (cv::Mat_<double>(3, 3)<<0.0, 1.0, 0.0, -1.0, 0.0, 1080.0, 0.0, 0.0, 1.0);
const cv::Mat ThreadContext::stableVec2RR = (cv::Mat_<double>(3, 3)<<0.0, -1.0, 1080.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
const cv::Mat ThreadContext::inmat=(cv::Mat_<double>(3, 3)<<1430.2,0.0,505.7, 0.0,1422.9,922.1,0.0,0.0,1.0);
std::queue<cv::Vec3d> ThreadContext::rTheta;
void ThreadContext::Init() {
    Release();
    out_semaphore_ = new MySemaphore(0);
    cm_semaphore_ = new MySemaphore(0);
    read_semaphore_ = new MySemaphore(20);
}
void ThreadContext::Release() {
    if(read_semaphore_ != nullptr){
        delete read_semaphore_;
    }
    if(out_semaphore_ != nullptr){
        delete out_semaphore_;
    }
    if(cm_semaphore_ != nullptr){
        delete cm_semaphore_;
    }
}