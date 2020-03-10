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
        void worker();
        Mat inmat=(cv::Mat_<double>(3, 3)<<1493.9,0.0,522.0, 0.0,1494.8,982.4,0.0,0.0,1.0);
        void gaussSmooth(Mat *rsOutTheta);
        Mat constantMulMat(Mat &src, double num);
    public:
        Size videosize;
        void start();
        void getRollingShutterR();
        vector<double> getTimeStampInFrame(double timestart, double timeend, int num);
        vector<double> interpolation (vector<double> x0, vector<double> x, vector<double> y);
        void getMatInFrame(Mat *rsOutTheta, vector<double> gyroInfoInFrameX, vector<double> gyroInfoInFrameY,
                           vector<double> gyroInfoInFrameZ);
        ~ThreadRollingShutter();



    };
}



#endif //STABLE_CAMERA_THREADROLLINGSHUTTER_H
