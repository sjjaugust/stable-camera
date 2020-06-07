//
// Created by 张哲华 on 19/09/2017.
//

#include "StableProcessor.h"
#include <android/log.h>

using namespace threads;

StableProcessor::~StableProcessor() {
    if (cm_thread_ != nullptr) {
        delete cm_thread_;
        cm_thread_ = nullptr;
    }
}

void StableProcessor::Init(Size videoSize) {
    ThreadContext::Init();
    buffer_index_ = 0;
    frame_index_ = 0;
    cm_thread_ = new ThreadCompensation();
    rs_thread_ = new ThreadRollingShutter();

    out_index_ = 0;
    cm_thread_ -> videoSize = videoSize;
    cm_thread_ -> cropControlFlag = true;//在此设置是否进行裁剪控制
    cm_thread_ -> shakeDetect = false;//在此设置是否进行抖动检测
    cm_thread_ -> cropRation = 0.85;

    //klt_thread_->start();
    cm_thread_->start();
    rs_thread_->start();

}

void StableProcessor::enqueueInputBuffer(int buffer_index, const Mat* new_frame, const Mat* RR, const Mat* rs_out_mat) {


    assert (buffer_index == buffer_index_);
    __android_log_print(ANDROID_LOG_DEBUG, "StableProcessor", "idx:%f",new_frame->at<double>(0, 1));
    new_frame->copyTo(ThreadContext::frameVec[buffer_index_]);
    RR->copyTo(ThreadContext::stableRVec[buffer_index_]);

    rs_out_mat->copyTo(ThreadContext::rs_out_theta_[buffer_index_]);
    if(is_first_frame_)
    {
        is_first_frame_ = false;
    } else
    {
//        ThreadContext::out_semaphore->Signal();
        ThreadContext::mc_semaphore->Signal();
    }


    buffer_index_ = (buffer_index_ + 1) % ThreadContext::BUFFERSIZE;

}

int StableProcessor::dequeueInputBuffer() {
    ThreadContext::read_semaphore->Wait();
    return buffer_index_;
}


void StableProcessor::enqueueOutputBuffer() {
    ThreadContext::read_semaphore->Signal();
}

void StableProcessor::dequeueOutputBuffer(Mat* const stableVec, Mat* const frame, Mat* const rsMat) {

    ThreadContext::out_semaphore->Wait();

    if (ThreadContext::stableTransformVec[0].cols == 0) {
        __android_log_print(ANDROID_LOG_ERROR, "NStableProcessor", "stableTransformVec[0] is empty");
    }
//    frame_index_ ++;

    ThreadContext::stableTransformVec[out_index_].copyTo(*stableVec);
//    cv::Mat stableVecc = cv::Mat::eye(3, 3, CV_64F);
//    stableVecc.copyTo(*stableVec);
    ThreadContext::frameVec[out_index_].copyTo(*frame);

    Mat outTemp(30,3,CV_64F);
    int tempPosition = 0;
    for(int i = 0; i < ThreadContext::KRsStripNum_; i++){
        Mat temp(3, 3, CV_64F);
        ThreadContext::rs_Mat_[out_index_][i].copyTo(temp);
        for(int j = tempPosition, l = 0; j < tempPosition+3; j++, l++){
            for(int k = 0; k < 3; k++){
                outTemp.at<double>(j, k) = temp.at<double>(l, k);
            }
        }
//        cv::Mat temp = cv::Mat::eye(3, 3, CV_64F);
//        for(int j = tempPosition, l = 0; j < tempPosition+3; j++, l++){
//            for(int k = 0; k < 3; k++){
//                outTemp.at<double>(j, k) = temp.at<double>(l, k);
//            }
//        }

        tempPosition+=3;
    }
    outTemp.copyTo(*rsMat);

    out_index_ = (out_index_ + 1) % ThreadContext::BUFFERSIZE;
//    putText(*frame, "(0,0)", Point(100,100), cv::FONT_HERSHEY_PLAIN, 3.0, Scalar(0), 10);
//    putText(*frame, "(1920,1080)", Point(1600,900), cv::FONT_HERSHEY_PLAIN, 3.0, Scalar(0), 10);

}

void StableProcessor::setCrop(bool isCrop)
{
    cm_thread_ -> cropControlFlag = isCrop;
}

void StableProcessor::setDrawStatus(bool is_draw) {
    cm_thread_ -> drawFlag = is_draw;
}