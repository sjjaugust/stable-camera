//
// Created by ShiJJ on 2020/6/15.
//

#ifndef VIDEO_STAB_RT_AUTOFILTER_H
#define VIDEO_STAB_RT_AUTOFILTER_H

#include <deque>
#include <vector>
#include <queue>
#include <opencv2/opencv.hpp>
#include <fstream>
#include "BSpline.h"
#include <android/log.h>

#define PI (2 * acos(0))
#define filterWeight(k,theta) exp(-(k) * (k) / 2.0 / (theta) / (theta)) / sqrt(2 * PI * (theta))
typedef struct limit{
    double xmin, xmax, ymin, ymax;
}limit;
class AutoFilter {
private:
    std::deque<cv::Mat> input_buffer_;
    std::queue<cv::Mat> output_buffer_;
    std::vector<cv::Mat> window_;
    double crop_rate_ = 0.75;
    int max_size_;
    double sigma_;
    int delay_num_ = 10;
    std::vector<double> weight_vec_;
    cv::Size size_;
    cv::Mat cropvertex_;
    cv::Mat vertex_;
    std::deque<cv::Mat> global_trans_;
    //new idea
    static int predict_num_;
    double que_x_[5], que_y_[5];
    int num_que_ = 0;
    double f_num_[5];
    double ex_count = 0;
    double cur_x = 0, cur_y = 0;
    bool need_fit_x_ = false;
    bool need_fit_y_ = false;
    std::queue<cv::Mat> s_mat_;
    std::vector<double> gauss_;
    double index_ = 0;
    std::queue<limit> limit_que_;


    void queue_in(double q[], int m, double x);
    void polyfit(double arrX[], double arrY[], int num, int n, double* result);
    double calError(double* ori, double* aft, int n);

    bool putIntoWindow(int target, int offset = 0);
    bool isInside(cv::Mat cropvertex ,cv::Mat newvertex);
    void processCrop(const cv::Mat& comp, const cv::Size& size);
public:
    explicit AutoFilter(int max_size = 30, double sigma = 40);
    bool push(cv::Mat goodar);
    cv::Mat pop();
};


#endif //VIDEO_STAB_RT_AUTOFILTER_H
