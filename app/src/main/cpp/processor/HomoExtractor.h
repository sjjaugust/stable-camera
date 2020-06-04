//
// Created by ShiJJ on 2020/6/1.
//

#ifndef STABLE_CAMERA_HOMOEXTRACTOR_H
#define STABLE_CAMERA_HOMOEXTRACTOR_H


#include <opencv2/opencv.hpp>
#include "ThreadContext.h"
#include <android/log.h>
using namespace threads;
#define PI 3.1415926535
class HomoExtractor {
private:
    std::vector<cv::Point2f> lastFeatures,curFeatures;
    std::vector<cv::Point2f> curFeaturesTmp, lastFeaturesTmp;
    cv::Mat lastGray,curGray;
    std::vector<uchar> status,status_choose;
    int ex_index_ = 0;
    int point_num[16];
    int statussize;
    cv::Mat H, H_ori, second_H;
    double sumup_err, sumup_H_err1, sumup_H_err2;
    int addp_frame = 0;
    bool stable_move=true, stable_move2=false;
    std::vector<cv::Mat> hom_s,homo_n,homo_a1,homo_a2;
    std::vector<int> block_index_;

    void detectFeature();
    void trackFeature(const cv::Mat& img1, const cv::Mat& img2);
    std::vector<cv::Point2f> detectCertainFeature(const std::vector<int>& block_index);
    void trackCertainFeature( std::vector<cv::Point2f>& last_add_feature);
    double point_distance(cv::Point2f p1,cv::Point2f p2);
    bool outOfImg(const cv::Point2f &point, const cv::Size &size);
    int in_area(cv::Point2f p1, int w, int h);
    bool judge_area();
    void calcul_Homo(std::vector<char> &ifselect, int niter, int type);
    bool judge_recal_simple(cv::Mat img1, std::vector<char> ifselect);
    double calcul_H_error(int c_h, int c_w);
    cv::Point2f goround(cv::Point2f p1, cv::Point2f p0, double degree);
    cv::Point2f goscale(cv::Point2f p1,cv::Point2f p0, double scale);
    double vec_cos(cv::Point2f s, cv::Point2f e1, cv::Point2f e2);
    void stab_feature_25(cv::Mat img1, cv::Mat img2);
    void stab_feature_25_H(cv::Mat img1, cv::Mat img2);


public:
    cv::Mat extractHomo( cv::Mat& img1, cv::Mat& img2);
};


#endif //STABLE_CAMERA_HOMOEXTRACTOR_H
