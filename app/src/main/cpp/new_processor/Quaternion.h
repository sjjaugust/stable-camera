//
// Created by ShiJJ on 2020/4/15.
//

#ifndef STABLE_CAMERA_QUATERNION_H
#define STABLE_CAMERA_QUATERNION_H

#include <cmath>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class Quaternion {
public:
    double w_, x_, y_, z_;
public:
    Quaternion(const double w = 0, const double x = 0, const double y = 0, const double z = 0);
    Quaternion Add(const Quaternion& other) const ;
    Quaternion Sub(const Quaternion& other) const ;
    Quaternion Mutiply(const Quaternion& other) const ;
    Quaternion Conjugate() const ;
    double Nrom() const ;
    Quaternion Inv() const ;
    Quaternion Normalize() const ;
    static Quaternion EulerToQuaternion(const double roll, const double pitch ,const double hdg);
    static cv::Mat QuaternionToR(const Quaternion& q);
    static Quaternion Slerp(const Quaternion& q1, const Quaternion& q2, const double t);
    static double Dot(const Quaternion& q1, const Quaternion& q2);
    static Quaternion CalAverage(const std::vector<Quaternion>& q_vector, const std::vector<double>& weight);
    static Quaternion Q1ToQ2(const Quaternion& q1, const Quaternion& q2);
    static cv::Vec3d Rotation(const Quaternion& q, const cv::Vec3d& vec);

public:
    Quaternion operator+(const Quaternion& other) const;
    Quaternion operator-(const Quaternion& other) const;
    Quaternion operator*(const Quaternion& other) const;
};


#endif //STABLE_CAMERA_QUATERNION_H
