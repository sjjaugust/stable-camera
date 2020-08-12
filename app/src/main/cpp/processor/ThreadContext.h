//
// Created by 张哲华 on 20/09/2017.
//

#ifndef VIDEOSTABLE_THREADCONTEXT_H
#define VIDEOSTABLE_THREADCONTEXT_H

#include "MySemaphore.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <list>

#define pi 3.1415926

using namespace std;
using namespace cv;
namespace threads {
    class ThreadContext {
    public:
        const static int SEGSIZE = 4;
        const static int BUFFERSIZE = SEGSIZE * 4;
        const static int MIN_DISTANCE = 20;
        constexpr const static float TRANSLATE_AMPLITUDE = 0.4f;
        constexpr const static float ROTATE_AMPLITUDE = 0.02f;
        const static int DOWNSAMPLE_SCALE = 4;

        static MySemaphore* read_semaphore;
        static MySemaphore* klt_semaphore;
        static MySemaphore* mc_semaphore;
        static MySemaphore* out_semaphore;

        static vector<Point2i> kltList;
        static list<list<vector<Point2f>>> trj;
        static queue<Vec<double, 3>> rTheta;
        static Mat frameVec[BUFFERSIZE];
        static Mat stableTransformVec[BUFFERSIZE];
        static Mat stableRVec[BUFFERSIZE];
        static std::queue<std::vector<cv::Point2f>> feature_by_r_;

        static void Init();
        static void Release();

    public:
        static MySemaphore* rs_semaphore_;
        static const int KRsStripNum_ = 10;
        static cv::Mat rs_out_theta_[BUFFERSIZE];
        static cv::Mat rs_Mat_[BUFFERSIZE][KRsStripNum_];
        static double guass_weight_[11];
        static void CreateGaussWeight(double* gauss_weight);
////************************测试**********************////
    public:
        static std::queue<cv::Mat> r_convert_que;
        static std::queue<cv::Mat> r_convert_que1;
        static cv::Mat last_old_Rotation_;
        static std::queue<double> gyro_z_theta_que;
        static std::queue<double> frame_time_que;
    };
}


#endif //VIDEOSTABLE_THREADCONTEXT_H
