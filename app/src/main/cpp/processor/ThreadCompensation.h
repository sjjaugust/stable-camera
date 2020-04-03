//
// Created by 张哲华 on 19/09/2017.
//

#ifndef VIDEOSTABLE_THREADCOMPENSATION_H
#define VIDEOSTABLE_THREADCOMPENSATION_H

#include <thread>
#include <cmath>
#include "ThreadContext.h"
#include "Filter.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <list>

namespace threads {
    class ThreadCompensation {
    public:
        Size videoSize, frameSize;
        bool cropControlFlag = true;//裁剪控制
        float cropRation;//裁剪率
        bool shakeDetect;//抖动检测
    private:
        const char* TAG = "CompensationThread";
        thread worker_thread_;
        int cm_las_index_ = 0;
        int cm_cur_index_ = 1;
        int ex_index_ = 0;
        int out_index_ = 0;
        bool with_roll = false;
        Filter filter;
        Mat curGray, lastGray;
        Mat H_scale;
        std::vector<Point2f> lastFeatures,curFeatures;
        std::vector<Point2f> lastFeaturesTmp,curFeaturesTmp;
        std::vector<uchar> status,status_choose;
        Vec<double, 3> lastRot;

        void worker();
        void detect_feature();
        void track_feature();
        Mat calcul_Homo(int niter);
        bool stable_count(double e);
        Mat computeAffine();
        void frameCompensate();
        bool cropControl( float cropr , Size size , Mat &affine );
        void calcul_Homo_s(std::vector<cv::Point2f> last, std::vector<cv::Point2f> cur);
        bool isInsideAfterTransform( Mat &affine , vector<Point2f> &pt_crop , vector<Point2f> &pt );
        bool affPointSimplify( vector<Point2f> &last_out , vector<Point2f> &cur_out );
        bool affPointSimplify_tri( vector<Point2f> &last_out , vector<Point2f> &cur_out );
        Mat moveAndScale();
        double computeMaxDegree( vector<Point2f> img_line , vector<Point2f> crop_line , double degree , Point2f center );

    public:
        void start();

        ~ThreadCompensation();
    };
}


#endif //VIDEOSTABLE_THREADCOMPENSATION_H
