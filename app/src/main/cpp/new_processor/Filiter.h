//
// Created by ShiJJ on 2020/4/18.
//

#ifndef STABLE_CAMERA_FILITER_H
#define STABLE_CAMERA_FILITER_H

#include <queue>
#include <deque>
#include <vector>
#include "Quaternion.h"

#define PI (2 * acos(0))
#define filterWeight(k,theta) exp(-(k) * (k) / 2.0 / (theta) / (theta)) / sqrt(2 * PI * (theta))
class Filiter {
private:
    std::queue<Quaternion> out_queue_;
    int kerner_;
    double sigma_;
    std::deque<Quaternion> window_;
    std::vector<Quaternion> trans_window_;
    double weight_sum_;
    std::vector<double> weight_vec_;
    std::vector<double> k_;
    void (*transform_func_)(const std::deque<Quaternion>& window, std::vector<Quaternion>& transWindow,int curframe);

public:
    explicit Filiter(int kernel = 5, double sigma = 2.9949,
            void (*transform_func_)(const std::deque<Quaternion>& window,
                    std::vector<Quaternion>& transWindow,int curframe) = NullFunc);
    bool Push(const Quaternion& data);
    Quaternion Pop();
    bool Empty();
    void IncFilter(int cur_frame, int offset);
    static void NullFunc(const std::deque<Quaternion>& window, std::vector<Quaternion>& transwindow,int curframe);
    static void Method1(const std::deque<Quaternion>& window, std::vector<Quaternion>& transwindow,int curframe);
};


#endif //STABLE_CAMERA_FILITER_H
