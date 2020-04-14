//
// Created by ShiJJ on 2020/4/13.
//

#ifndef STABLE_CAMERA_THETAHELPER_H
#define STABLE_CAMERA_THETAHELPER_H

#include <vector>
#include <opencv2/opencv.hpp>

class ThetaHelper {
private:
    std::vector<double> angular_x_;
    std::vector<double> angular_y_;
    std::vector<double> angular_z_;
    std::vector<double> timestamp_gyro_;
    std::vector<double> timestamp_frame_;
    unsigned long long gyro_index_
    unsigned long long frame_index_

    double last_x_, last_y_, last_z_;
    cv::Vec<double, 4> last_theta_;
public:
    void Init();
    void GetQuaternion(const double timestamp, cv::Mat* mat);
    void GetEuler(std::vector<cv::Vec<double, 4>>& euler_theta);
    void PutValue(const double timestamp, const double x, const double y, const double z);
    void EulerToQuaternion()
};


#endif //STABLE_CAMERA_THETAHELPER_H
