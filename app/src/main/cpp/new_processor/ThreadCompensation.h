//
// Created by ShiJJ on 2020/4/18.
//

#ifndef STABLE_CAMERA_THREADCOMPENSATION_H
#define STABLE_CAMERA_THREADCOMPENSATION_H

#include <thread>
#include "ThreadContext.h"
#include "Filiter.h"

class ThreadCompensation {
private:
    std::thread work_thread_;
    int cm_index_ = 0;
    int rs_index = 0;
    int out_index_ = 0;
    Filiter filiter;
    std::queue<Quaternion> q_cache_;
    const double crop_ratio_ = 0.95;
    cv::Size frame_size_;
    bool is_first = true;
    Quaternion nn;
    Quaternion last;
    std::queue<std::vector<Quaternion>> rs_q_cache_;
public:
    bool crop_control_flag = true;
private:
    void Work();
    void FrameCompensation();
    bool CropControl(double crop_ratio, const cv::Size& size, cv::Mat& mat);
    bool IsInside(cv::Mat crop_vertex, cv::Mat new_vertex);
    void RollingShutter(int cm_index);
    std::vector<double> GetTimeStampInFrame(double timestart, double timeend,
                                            int num);
public:
    void Start();
    ~ThreadCompensation();
};


#endif //STABLE_CAMERA_THREADCOMPENSATION_H
