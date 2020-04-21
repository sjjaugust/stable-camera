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
public:
    bool crop_control_flag = true;
private:
    void Work();
    void FrameCompensation();
public:
    void Start();
    ~ThreadCompensation();
};


#endif //STABLE_CAMERA_THREADCOMPENSATION_H
