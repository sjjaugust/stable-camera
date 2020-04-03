//
// Created by 佳佳 on 2019-05-13.
//

#include "Filter.h"

#define PI (2 * acos(0))
#define filterWeight(k,theta) exp(-(k) * (k) / 2.0 / (theta) / (theta)) / sqrt(2 * PI * (theta))

void Filter::null_func(const deque<Mat>& window, vector<Mat>& transWindow, int curframe) {
    //cout<<"!!!!!!!!null_func"<<endl;
    transWindow.clear();
    transWindow.resize(window.size());
    for (int i = 0; i < window.size(); i ++) {
        
        transWindow[i] = window[i];
    }
    
}

void Filter::delta_T(const deque<Mat>& window, vector<Mat>& transWindow ,int curframe) {
    //cout<<"!!!!!!!!delta_T"<<endl;
    int kernel =  curframe;
    transWindow.clear();
    transWindow.resize(window.size());
    transWindow[kernel] = Mat::eye(3, 3, CV_64F);
    for (int i = kernel - 1; i >= 0; i --) {
        transWindow[i] = window[i].inv() * transWindow[i + 1];
    }
    for (int i = kernel + 1; i < window.size(); i ++) {
        transWindow[i] = window[i - 1] * transWindow[i - 1];
    }
}

bool Filter::push(Mat data) {
    if (data.empty()) {
        int curframe=_kernel/2;
        while (_window.size()>_kernel/2) {
            
            inc_filter(curframe);
            _window.pop_front();
            
        }
        return true;
    }
    _window.push_back(data);
    
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

Mat Filter::pop() {
    if (!_outputBuffer.empty()) {
        Mat ret = _outputBuffer.front().clone();
        _outputBuffer.pop();
        return ret;
    }
    return Mat();
}

Filter::Filter(int kernel, double sigma, void (*transform)(const deque<Mat>& window, vector<Mat>& transWindow, int curframe))
: _kernel(kernel), _sigma(sigma), _transform(transform) {
    _kernelSum = 0;
    for (int i = 0; i < kernel; i ++) {
        double k=i-kernel/2.0;
        _k.push_back(k);
        
        double weight = filterWeight(k, sigma);
        _kernelVec.push_back(weight);
        //cout << "weight:"<<weight << endl;
        _kernelSum += weight;
    }
    _transformedWindow.resize(_kernel);
}

void Filter::inc_filter( int curframe, int kernelOffset) {
    //cout << "====== window ======" << endl;
    //for (int i = 0; i < _window.size(); i ++) {
    //    cout << _window[i] << endl;
    //}
    //int curframe=_window.size()/2;
    _transform(_window, _transformedWindow,curframe);
    //cout << "====== transformed ======" << endl;
    //for (int i = 0; i < _transformedWindow.size(); i ++) {
    //    cout << _transformedWindow[i] << endl;
    //}
    //cout << endl;
    Mat sumMat = Mat::zeros(3, 3, CV_64F);
    _kernelSum=0;
    //cout<<"!!!!!!!!!!"<<_transformedWindow.size()<<endl; //30...29.28.27...4.3.2
    for (int j = 0; j < _transformedWindow.size(); j ++) {
        sumMat += _kernelVec[j + kernelOffset] * _transformedWindow[j];
        _kernelSum+=_kernelVec[j+ kernelOffset];
    }
    sumMat /= _kernelSum;
    _outputBuffer.push(sumMat);
}

bool Filter::empty() {
    return _outputBuffer.empty();
}
