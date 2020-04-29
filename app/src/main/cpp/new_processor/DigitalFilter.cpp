//
// Created by ShiJJ on 2020/4/29.
//

#include "DigitalFilter.h"
#define PI (2 * acos(0))
#define filterWeight(k,theta) exp(-(k) * (k) / 2.0 / (theta) / (theta)) / sqrt(2 * PI * (theta))

void DigitalFilter::null_func(const std::deque<cv::Mat>& window, std::vector<cv::Mat>& transWindow, int curframe) {
    //cout<<"!!!!!!!!null_func"<<endl;
    transWindow.clear();
    transWindow.resize(window.size());
    for (int i = 0; i < window.size(); i ++) {

        transWindow[i] = window[i];
    }

}

void DigitalFilter::delta_T(const std::deque<cv::Mat>& window, std::vector<cv::Mat>& transWindow ,int curframe) {
    //cout<<"!!!!!!!!delta_T"<<endl;
    int kernel =  curframe;
    transWindow.clear();
    transWindow.resize(window.size());
    transWindow[kernel] = cv::Mat::eye(3, 3, CV_64F);
    for (int i = kernel - 1; i >= 0; i --) {
        transWindow[i] = window[i].inv() * transWindow[i + 1];
    }
    for (int i = kernel + 1; i < window.size(); i ++) {
        transWindow[i] = window[i - 1] * transWindow[i - 1];
    }
}
bool DigitalFilter::push(cv::Mat data) {
    if (data.empty()) {
        int curframe=_kernel/2;
        while (_window.size()>_kernel/2) {

            inc_filter(curframe);
            _window.pop_front();

        }
        return true;
    }
    _window.push_back(data);//仿射矩阵放入_window

    if (_window.size() < _kernel) {

        if (_window.size()>=_kernel/2)         //size>=15
        {
            int curframe=_window.size()-(_kernel/2);
            int offset=_kernel-_window.size();
            inc_filter(curframe,offset);
            return true;
        }
        else
        {
            return false;
        }
    } else {
        int curframe=_window.size()/2;
        inc_filter(curframe);
        _window.pop_front();
        return true;
    }
}

cv::Mat DigitalFilter::pop() {
    if (!_outputBuffer.empty()) {
        cv::Mat ret = _outputBuffer.front().clone();
        _outputBuffer.pop();
        return ret;
    }
    return cv::Mat();
}

DigitalFilter::DigitalFilter(int kernel, double sigma, void (*transform)(const std::deque<cv::Mat>& window, std::vector<cv::Mat>& transWindow, int curframe))
        : _kernel(kernel), _sigma(sigma), _transform(transform) {
    _kernelSum = 0;
    for (int i = 0; i < kernel; i ++) {
        double k=i-kernel/2.0;
        _k.push_back(k);

        double weight = filterWeight(k, sigma);
        _kernelVec.push_back(weight);
        //cout << "weight:"<<weight << endl;
        _kernelSum += weight;
    }//生成高斯核并计算权值之和
    _transformedWindow.resize(_kernel);//调整窗口个数
}

void DigitalFilter::inc_filter( int curframe, int kernelOffset) {
    //cout << "====== window ======" << endl;
    //for (int i = 0; i < _window.size(); i ++) {
    //    cout << _window[i] << endl;
    //}
    //int curframe=_window.size()/2;
    _transform(_window, _transformedWindow,curframe);//把_window中的仿射矩阵复制到_transformedWindow中
    //cout << "====== transformed ======" << endl;
    //for (int i = 0; i < _transformedWindow.size(); i ++) {
    //    cout << _transformedWindow[i] << endl;
    //}
    //cout << endl;
    cv::Mat sumMat = cv::Mat::zeros(3, 3, CV_64F);
    _kernelSum=0;
    //cout<<"!!!!!!!!!!"<<_transformedWindow.size()<<endl; //30...29.28.27...4.3.2
    for (int j = 0; j < _transformedWindow.size(); j ++) {
        sumMat += _kernelVec[j + kernelOffset] * _transformedWindow[j];
        _kernelSum+=_kernelVec[j+ kernelOffset];
    }
    sumMat /= _kernelSum;//归一化？让系数归一
    _outputBuffer.push(sumMat);
}

bool DigitalFilter::empty() {
    return _outputBuffer.empty();
}
