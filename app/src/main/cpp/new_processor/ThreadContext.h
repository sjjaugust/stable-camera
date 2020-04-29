//
// Created by ShiJJ on 2020/4/13.
//

#ifndef STABLE_CAMERA_THREADCONTEXT_H
#define STABLE_CAMERA_THREADCONTEXT_H


#include <opencv2/core/mat.hpp>
#include "MySemaphore.h"
#include <queue>
#include "ThreadContext.h"
class ThreadContext {
public:
    const static int MIN_DISTANCE = 20;
    constexpr const static float TRANSLATE_AMPLITUDE = 0.4f;
    constexpr const static float ROTATE_AMPLITUDE = 0.02f;
    const static int DOWNSAMPLE_SCALE = 4;

    static const cv::Mat RR2stableVec ;
    static const cv::Mat stableVec2RR ;
    static const cv::Mat inmat;
    static const int BUFFER_SIZE_;
    static cv::Mat frame_vec_[];
    static cv::Mat quaternion_vec_[];
    static cv::Mat stable_vec_[];
    static MySemaphore* read_semaphore_;
    static MySemaphore* out_semaphore_;
    static MySemaphore* cm_semaphore_;
    static MySemaphore* rs_semaphore_;
    static cv::Mat rs_convert_mat_;
    static std::queue<cv::Vec3d> rTheta;
public:
    static void Init();
    static void Release();
};


#endif //STABLE_CAMERA_THREADCONTEXT_H
