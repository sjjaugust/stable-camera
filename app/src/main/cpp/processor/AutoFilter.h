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

#define PI (2 * acos(0))
#define filterWeight(k,theta) exp(-(k) * (k) / 2.0 / (theta) / (theta)) / sqrt(2 * PI * (theta))
class AutoFilter {
private:
    std::deque<cv::Mat> input_buffer_;
    std::queue<cv::Mat> output_buffer_;
    std::vector<cv::Mat> window_;
    double crop_rate_ = 0.7;
    int max_size_;
    double sigma_;
    int delay_num_ = 10;
    std::vector<double> weight_vec_;
    cv::Size size_;
    cv::Mat cropvertex_;
    cv::Mat vertex_;
    std::deque<cv::Mat> global_trans_;
    //new idea
    double x_q[50], y_q[50];
    double f_num[50];
    int q_size = 30;
    int q_pow = 2;
    int count_ = 0;

    cv::Mat processTrans(const cv::Mat& comp, const cv::Size& size);
    void queue_in(double q[], int m, double x);
    double polyfit(double arrX[],double arrY[],int num,int n,double x);

    void putIntoWindow(int target, int offset = 0);
    bool isInside(cv::Mat cropvertex ,cv::Mat newvertex);
    cv::Mat calGlobalTrans(const cv::Mat& comp, double crop_rate, const cv::Size& videoSize);
    cv::Mat processGlobalTrans(const cv::Mat& comp, int target, int offset);

public:
    explicit AutoFilter(int max_size = 30, double sigma = 40);
    bool push(cv::Mat goodar);
    cv::Mat pop();
};


#endif //VIDEO_STAB_RT_AUTOFILTER_H
