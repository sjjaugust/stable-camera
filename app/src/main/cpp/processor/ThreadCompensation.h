//
// Created by 张哲华 on 19/09/2017.
//

#ifndef VIDEOSTABLE_THREADCOMPENSATION_H
#define VIDEOSTABLE_THREADCOMPENSATION_H

#include <thread>
#include <cmath>
#include "ThreadContext.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <list>

namespace threads {
    class ThreadCompensation {
    public:
        Size videoSize;
        bool cropControlFlag = false;//裁剪控制
        float cropRation;//裁剪率
        bool shakeDetect;//抖动检测
    private:
        const char* TAG = "CompensationThread";
        thread worker_thread_;
        void worker();
        void computeAffine( vector<Point2f> &avgFeatPos , vector<Mat> &affineMatrix );
        void frameCompensate();
        bool cropControl( float cropRation , Point2f center , Point2f &shift , double &degree );
        bool isInsideAfterTransform( Mat &affine , vector<Point2f> &pt_crop , vector<Point2f> &pt );
        double computeMaxDegree( vector<Point2f> img_line , vector<Point2f> crop_line , double degree , Point2f center );

    public:
        void start();
        ~ThreadCompensation();
    };
}


#endif //VIDEOSTABLE_THREADCOMPENSATION_H
