//
// Created by 佳佳 on 2019-05-13.
//

#ifndef VIDEOSTABILIZATION_FILTER_H
#define VIDEOSTABILIZATION_FILTER_H

#include <opencv2/opencv.hpp>
#include <android/log.h>
#include "../include/opencv2/core/mat.hpp"

using namespace cv;
using namespace std;

class Filter {
private:
    queue<Mat> _outputBuffer;//存放平滑好的仿射矩阵
    
    void inc_filter(int curframe =17, int kernelOffset = 0);  //窗口
    int _kernel;//使用几帧数据进行平滑
    double _sigma;//高斯核参数
    deque<Mat> _window;//存放仿射矩阵
    vector<Mat> _transformedWindow;//从_window复制过来
    double _kernelSum;//存放权值之和，为进行归一化
    vector<double> _kernelVec;//存放权值
    vector<double> _k;//存放高斯核系数
    void (*_transform)(const deque<Mat>& window, vector<Mat>& transWindow,int curframe);//将window复制到transWindow
    
    
public:
    explicit Filter(int kernel = 5, double sigma = 2.44949, void (*transform_)(const deque<Mat>& window, vector<Mat>& transWindow,int curframe) = null_func);
    static void delta_T(const deque<Mat>& window, vector<Mat>& transWindow,int curframe);//一种预处理仿射矩阵序列的处理方式
    static void null_func(const deque<Mat>& window, vector<Mat>& transWindow,int curframe);//默认方式
    bool push(Mat data);
    Mat pop();
    bool empty();
};


#endif //VIDEOSTABILIZATION_FILTER_H
