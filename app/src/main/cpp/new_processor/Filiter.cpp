//
// Created by ShiJJ on 2020/4/18.
//

#include "Filiter.h"
Filiter::Filiter(int kernel, double sigma,
                 void (*transform_func)(const std::deque<Quaternion> &, std::vector<Quaternion> &, int))
                 : kerner_(kernel), sigma_(sigma), transform_func_(transform_func){
    weight_sum_ = 0;
    for(int i = 0; i < kernel; i++){
        double k = i - kernel / 2.0;
        k_.push_back(k);
        double weight = filterWeight(k, sigma);
        weight_vec_.push_back(weight);
        weight_sum_ += weight;
    }
    trans_window_.resize((unsigned int)kernel);
}
bool Filiter::Push(const Quaternion &data) {
    window_.push_back(data);
    if(window_.size() < kerner_){
        if(window_.size() >= kerner_/2){
            int cur_frame = window_.size() - kerner_ / 2;
            int offset = kerner_ - window_.size();
            IncFilter(cur_frame, offset);
            return true;
        } else {
            return false;
        }
    } else {
        int cur_frame = kerner_ / 2;
        IncFilter(cur_frame, 0);
        window_.pop_front();
        return true;
    }
}
Quaternion Filiter::Pop() {
    if(!out_queue_.empty()){
        Quaternion ret = out_queue_.front();
        out_queue_.pop();
        return ret;
    }
    return Quaternion(0, 0, 0, 0);
}
bool Filiter::Empty() {
    return out_queue_.empty();
}
void Filiter::NullFunc(const std::deque<Quaternion> &window, std::vector<Quaternion> &transwindow,
                       int curframe) {
    transwindow.clear();
    transwindow.resize(window.size());
    for(int i = 0; i < window.size(); i++){
        transwindow[i] = window[i];
    }
}
void Filiter::IncFilter(int cur_frame, int offset) {
    Quaternion result;
    transform_func_(window_, trans_window_, cur_frame);
    if(offset != 0){
        std::vector<double> weight_temp;
        for(int i = 0; i < trans_window_.size(); i++){
            weight_temp.push_back(weight_vec_[i]);
        }
        result = Quaternion::CalAverage(trans_window_, weight_temp);
    } else {
        result = Quaternion::CalAverage(trans_window_, weight_vec_);
    }
    out_queue_.push(result);
}
void Filiter::Method1(const std::deque<Quaternion> &window, std::vector<Quaternion> &transwindow,
                      int curframe) {
    transwindow.clear();
    transwindow.resize(window.size());
    transwindow[curframe] = Quaternion::EulerToQuaternion(0, 0, 0);
    for(int i = curframe - 1; i >= 0; i--){
        transwindow[i] = window[i] * window[curframe].Inv();
    }
    for(int i = curframe + 1; i< window.size(); i++){
        transwindow[i] = window[i] * window[curframe].Inv();
    }

}