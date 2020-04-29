//
// Created by ShiJJ on 2020/4/29.
//

#ifndef STABLE_CAMERA_DIGITALFILTER_H
#define STABLE_CAMERA_DIGITALFILTER_H


#include <opencv2/opencv.hpp>
#include <android/log.h>
#include "../include/opencv2/core/mat.hpp"
#include <queue>
#include <deque>

class DigitalFilter {
private:
    std::queue<cv::Mat> _outputBuffer;//存放平滑好的仿射矩阵

    void inc_filter(int curframe =17, int kernelOffset = 0);  //窗口
    int _kernel;//使用几帧数据进行平滑
    double _sigma;//高斯核参数
    std::deque<cv::Mat> _window;//存放仿射矩阵
    std::vector<cv::Mat> _transformedWindow;//从_window复制过来
    double _kernelSum;//存放权值之和，为进行归一化
    std::vector<double> _kernelVec;//存放权值
    std::vector<double> _k;//存放高斯核系数
    void (*_transform)(const std::deque<cv::Mat>& window, std::vector<cv::Mat>& transWindow,int curframe);//将window复制到transWindow


public:
    //2.44949
    explicit DigitalFilter(int kernel = 5, double sigma = 2.44949, void (*transform_)(const std::deque<cv::Mat>& window, std::vector<cv::Mat>& transWindow,int curframe) = null_func);
    static void delta_T(const std::deque<cv::Mat>& window, std::vector<cv::Mat>& transWindow,int curframe);//一种预处理仿射矩阵序列的处理方式
    static void null_func(const std::deque<cv::Mat>& window, std::vector<cv::Mat>& transWindow,int curframe);//默认方式
    bool push(cv::Mat data);
    cv::Mat pop();
    bool empty();
};


#endif //STABLE_CAMERA_DIGITALFILTER_H
