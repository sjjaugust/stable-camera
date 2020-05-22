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
#define LOG_TAG    "cThreadRollingShutter"
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
namespace threads{
    class ThreadRollingShutter{
    private:
        const char* TAG = "ThreadRollingShutter";
        thread work_thread;
//        cv::Mat inmat=(cv::Mat_<double>(3, 3)<<1430.2,0.0,505.7, 0.0,1422.9,922.1,0.0,0.0,1.0);//OnePlus 6T
        cv::Mat inmat=(cv::Mat_<double>(3, 3)<<1492.89950430177,0.0,940.850079740057, 0.0,1496.13805384036,552.228021875255,0.0,0.0,1.0);//demo board
        int buffer_index_ = 0;
        std::vector<double> angle_;
        const double correction_threshold = 0.1;
        static bool is_stable_;
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
        static void getStableStatus(bool is_stable);


    };
}



#endif //STABLE_CAMERA_THREADROLLINGSHUTTER_H
