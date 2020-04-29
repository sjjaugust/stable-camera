//
// Created by ShiJJ on 2020/4/16.
//

#ifndef STABLE_CAMERA_STABLEPROCESSOR_H
#define STABLE_CAMERA_STABLEPROCESSOR_H


#include <opencv2/core/mat.hpp>
#include "ThreadCompensation.h"

class StableProcessor {
private:
    volatile int in_index_ = 0;
    volatile int out_index_ = 0;
    ThreadCompensation* cm_thread_;
    bool is_first = true;
public:
    ~StableProcessor();
    void EnqueueInputBuffer(int buffer_index, const cv::Mat* new_frame, const cv::Mat* quaternion_mat);
    int DequeueInputBuffer();
    void DequeueOutputBuffer(cv::Mat* const stableVec, cv::Mat* const frame, cv::Mat* const rs_convert_mat);
    void EnqueueOutputBuffer();
    void SetCrop(bool is_crop);//TODO SetCrop
    void Init();
};


#endif //STABLE_CAMERA_STABLEPROCESSOR_H
