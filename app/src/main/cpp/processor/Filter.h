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
    queue<Mat> _outputBuffer;
    
    void inc_filter(int curframe =17, int kernelOffset = 0);  //窗口
    int _kernel;
    double _sigma;
    deque<Mat> _window;
    vector<Mat> _transformedWindow;
    double _kernelSum;
    vector<double> _kernelVec;
    vector<double> _k;
    void (*_transform)(const deque<Mat>& window, vector<Mat>& transWindow,int curframe);
    
    
public:
    explicit Filter(int kernel = 5, double sigma = 2.44949, void (*transform_)(const deque<Mat>& window, vector<Mat>& transWindow,int curframe) = null_func);
    static void delta_T(const deque<Mat>& window, vector<Mat>& transWindow,int curframe);
    static void null_func(const deque<Mat>& window, vector<Mat>& transWindow,int curframe);
    bool push(Mat data);
    Mat pop();
    bool empty();
};


#endif //VIDEOSTABILIZATION_FILTER_H
