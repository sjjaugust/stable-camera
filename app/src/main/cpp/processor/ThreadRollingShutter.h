//
// Created by ShiJJ on 2020/2/27.
//

#ifndef STABLE_CAMERA_THREADROLLINGSHUTTER_H
#define STABLE_CAMERA_THREADROLLINGSHUTTER_H

#include "ThreadContext.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include <thread>
#include <cmath>

namespace threads{
    class ThreadRollingShutter{
    private:
        const char* TAG = "ThreadRollingShutter";
        thread work_thread;
        const Mat inmat=(cv::Mat_<double>(3, 3)<<1430.2,0.0,505.7, 0.0,1422.9,922.1,0.0,0.0,1.0);
        int buffer_index_ = 0;
        std::vector<double> angle_;
        const double correction_threshold = 0.1;
    private:
        void worker();
        void gaussSmooth(cv::Mat *rsOutTheta);
        Mat constantMulMat(cv::Mat &src, double num);
    public:
        void start();
        void getRollingShutterR();
        vector<double> getTimeStampInFrame(double timestart, double timeend, int num);
        vector<double> interpolation (std::vector<double> x0, std::vector<double> x, std::vector<double> y);
        void getMatInFrame(cv::Mat *rsOutTheta, std::vector<double> gyroInfoInFrameX, std::vector<double> gyroInfoInFrameY,
                           std::vector<double> gyroInfoInFrameZ);
        ~ThreadRollingShutter();
        void showMat(cv::Mat cvMat);


    };
}



#endif //STABLE_CAMERA_THREADROLLINGSHUTTER_H
