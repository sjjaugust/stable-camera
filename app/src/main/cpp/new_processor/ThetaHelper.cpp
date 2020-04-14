//
// Created by ShiJJ on 2020/4/13.
//

#include "ThetaHelper.h"
void ThetaHelper::PutValue(const double timestamp, const double x, const double y,
                           const double z) {
    timestamp_gyro_.push_back(timestamp);
    angular_x_.push_back(y);
    angular_y_.push_back(-x);
    angular_z_.push_back(z);
}
void ThetaHelper::Init() {
    gyro_index_ = 0;
    frame_index_ = 0;
    angular_x_.clear();
    angular_y_.clear();
    angular_z_.clear();
}
void ThetaHelper::GetQuaternion(const double timestamp, cv::Mat *mat) {
    timestamp_frame_.push_back(timestamp);
    std::vector<cv::Vec<double, 4>> euler_theta;
    GetEuler(euler_theta);
}
void ThetaHelper::GetEuler(std::vector<cv::Vec<double, 4>>& euler_theta) {
    double frame_time = timestamp_frame_[frame_index_];
    double gyro_time = timestamp_gyro_[gyro_index_];
    cv::Vec<double, 4> temp = {0, 0, 0, 0};
    if(frame_index_ > 0){
        double last_time = timestamp_frame_[frame_index_ - 1];
        temp[1] = (-last_x_) * (gyro_time - last_time) + last_theta_[1];
        temp[2] = (-last_y_) * (gyro_time - last_time) + last_theta_[2];
        temp[3] = (-last_z_) * (gyro_time - last_time) + last_theta_[3];
    }
    while (gyro_time < frame_time){
        double gyro_time_next = timestamp_gyro_[gyro_index_ + 1];
        double angular_x = angular_x_[gyro_index_];
        double angular_y = angular_y_[gyro_index_];
        double angular_z = angular_z_[gyro_index_];
        if(gyro_time_next < frame_time){
            temp[0] = gyro_time;
            temp[1] += (-angular_x) * (gyro_time_next - gyro_time);
            temp[2] += (-angular_y) * (gyro_time_next - gyro_time);
            temp[3] += (-angular_z) * (gyro_time_next - gyro_time);
            euler_theta.push_back(temp);
            gyro_index_++;
        }else {
            temp[0] = gyro_time;
            temp[1] += (-angular_x) * (gyro_time_next - frame_time);
            temp[2] += (-angular_y) * (gyro_time_next - frame_time);
            temp[3] += (-angular_z) * (gyro_time_next - frame_time);
            euler_theta.push_back(temp);
            gyro_index_++;
            last_x_ = angular_x;
            last_y_ = angular_y;
            last_z_ = angular_z;
            last_theta_ = temp;
            break;
        }
        gyro_time = timestamp_gyro_[gyro_index_];
    }
    frame_index_++;

}