//
// Created by ShiJJ on 2020/4/15.
//

#include "Quaternion.h"
#include <android/log.h>

Quaternion::Quaternion(const double w, const double x, const double y, const double z)
        : w_(w), x_(x), y_(y), z_(z){
}
Quaternion Quaternion::Add(const Quaternion& other) const {
    return Quaternion(w_+other.w_, x_+other.x_, y_+other.y_, z_+other.z_);
}
Quaternion Quaternion::Sub(const Quaternion& other) const {
    return Quaternion(w_ - other.w_, x_ - other.x_, y_ - other.y_, z_ - other.z_);
}
Quaternion Quaternion::Mutiply(const Quaternion &other) const {
    double new_w = w_ * other.w_ - x_ * other.x_ - y_ * other.y_ - z_ * other.z_;
    double new_x = x_ * other.w_ + w_ * other.x_ - z_ * other.y_ + y_ * other.z_;
    double new_y = y_ * other.w_ + z_ * other.x_ + w_ * other.y_ - x_ * other.z_;
    double new_z = z_ * other.w_ - y_ * other.x_ + x_ * other.y_ + w_ * other.z_;
    return Quaternion(new_w, new_x, new_y, new_z);
}
Quaternion Quaternion::Conjugate() const {
    return Quaternion(w_, -x_, -y_, -z_);
}
double Quaternion::Nrom() const {
    return sqrt(w_ * w_ + x_ * x_ + y_ * y_ + z_ * z_);
}
Quaternion Quaternion::Normalize() const {
    double length = 1.0 / (this->Nrom() * this->Nrom());
    return Quaternion(w_ * length, x_ * length, y_ * length, z_ * length);
}
Quaternion Quaternion::Inv() const {
    Quaternion conjugate = this->Conjugate();
    double temp = this->Nrom() * this->Nrom();
    return Quaternion(conjugate.w_ / temp, conjugate.x_ / temp, conjugate.y_ / temp, conjugate.z_ / temp);
}
Quaternion Quaternion::EulerToQuaternion(const double roll, const double pitch, const double hdg) {
    Quaternion q_ret;
    double cosRoll = cos(roll  * 0.5);
    double sinRoll = sin(roll  * 0.5);
    double cosPitch = cos(pitch * 0.5);
    double sinPitch = sin(pitch * 0.5);
    double cosHeading = cos(hdg   * 0.5);
    double sinHeading = sin(hdg   * 0.5);
    q_ret.w_ = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q_ret.x_ = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q_ret.y_ = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q_ret.z_ = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
    return q_ret;
}
Quaternion Quaternion::Slerp(const Quaternion& q1, const Quaternion& q2, const double t) {
    Quaternion result;
    Quaternion start = q1;
    Quaternion end = q2;
    double cosa = start.w_ * end.w_ + start.x_ * end.x_ + start.y_ * end.y_
                  + start.z_ * end.z_;
    double k0, k1;
    if(cosa < 0.0){
        end.w_ = -end.w_;
        end.x_ = -end.x_;
        end.y_ = -end.y_;
        end.z_ = -end.z_;
        cosa = -cosa;
    }
    if(cosa > 0.9995){
        k0 = 1.0 - t;
        k1 = t;
    } else {
        double sina = sqrt(1.0 - cosa * cosa);
        double a = atan2(sina, cosa);
        k0 = sin((1.0 - t) * a) / sina;
        k1 = sin(t*a) / sina;
    }
    result.w_ = start.w_ * k0 + end.w_ * k1;
    result.x_ = start.x_ * k0 + end.x_ * k1;
    result.y_ = start.y_ * k0 + end.y_ * k1;
    result.z_ = start.z_ * k0 + end.z_ * k1;
    return result;

}
cv::Mat Quaternion::QuaternionToR(const Quaternion& q) {
    cv::Mat mat = cv::Mat(3, 3, CV_64F);
    double w = q.w_;
    double x = q.x_;
    double y = q.y_;
    double z = q.z_;
    mat.at<double>(0, 0) = 1 - 2 * y * y - 2 * z * z;
    mat.at<double>(0, 1) = 2 * x * y - 2 * z * w;
    mat.at<double>(0, 2) = 2 * x * z + 2 * y * w;
    mat.at<double>(1, 0) = 2 * x * y + 2 * z * w;
    mat.at<double>(1, 1) = 1 - 2 * x * x - 2 * z * z;
    mat.at<double>(1, 2) = 2 * y * z - 2 * x * w;
    mat.at<double>(2, 0) = 2 * x * z - 2 * y * w;
    mat.at<double>(2, 1) = 2 * y * z + 2 * x * w;
    mat.at<double>(2, 2) = 1 - 2 * x * x - 2 * y * y;
    return mat;
}
double Quaternion::Dot(const Quaternion &q1, const Quaternion &q2) {
    return q1.w_ * q2.w_ + q1.x_ * q2.x_ + q1.y_ * q2.y_ + q1.z_ * q2.z_;
}
Quaternion Quaternion::CalAverage(const std::vector<Quaternion>& q_vector, const std::vector<double>& weight) {
    double weight_sum = 0;
    cv::Mat M = cv::Mat::zeros(4, 4, CV_64F);
    cv::Mat e_value_mat;
    cv::Mat e_vector_mat;
    for(int i = 0; i < q_vector.size(); i++){
        double w = weight[i];
        weight_sum += w;
        cv::Vec4d q_vec(q_vector[i].w_, q_vector[i].x_, q_vector[i].y_, q_vector[i].z_);
        cv::Matx<double, 4, 4> mat_temp = q_vec * q_vec.t();
        cv::Mat mat(mat_temp);
        M = M + w * mat;
    }
    M = (1/weight_sum)*M;
    cv::eigen(M, e_value_mat, e_vector_mat);
    double max_eigen_value = e_value_mat.at<double>(0, 0);
    int max_row = 0;
    for(int i = 1; i < e_value_mat.rows; i++){
        if(e_value_mat.at<double>(i, 0) > max_eigen_value){
            max_eigen_value = e_value_mat.at<double>(i, 0);
            max_row = i;
        }
    }
    Quaternion result;
    result.w_ = e_vector_mat.at<double>(max_row, 0);
    result.x_ = e_vector_mat.at<double>(max_row, 1);
    result.y_ = e_vector_mat.at<double>(max_row, 2);
    result.z_ = e_vector_mat.at<double>(max_row, 3);
    return result;
}
Quaternion Quaternion::Q1ToQ2(const Quaternion &q1, const Quaternion &q2){
    return q1.Inv() * q2;
}
cv::Vec3d Quaternion::Rotation(const Quaternion &q, const cv::Vec3d &vec) {
    cv::Vec3d result;
    Quaternion vec_q(0, vec[0], vec[1], vec[2]);
    Quaternion result_q;
    result_q = q * vec_q * q.Inv();
    result[0] = result_q.x_;
    result[1] = result_q.y_;
    result[2] = result_q.z_;
    return result;
}
std::vector<Quaternion> Quaternion::Interpolation (const std::vector<double>& x0, const std::vector<double>& x,
                                                   const std::vector<Quaternion>& y){
    std::vector<Quaternion> out;
    for(auto xx0 : x0){
        Quaternion yy0 = Quaternion::EulerToQuaternion(0,0,0);
        for(int i = 1; i < x.size(); i++){
            if(xx0>x[i]) continue;
            else if(xx0==x[i]){
                yy0 = y[i];
                break;
            }
            if(i!=0){
                double ratio = (xx0 - x[i-1]) / (x[i] - x[i-1]);
                yy0 = Quaternion::Slerp(y[i-1], y[i], ratio);
                break;
            }
        }
        out.push_back(yy0);
    }
    return out;
}
Quaternion Quaternion::operator+(const Quaternion &other) const {
    return Add(other);
}
Quaternion Quaternion::operator-(const Quaternion &other) const {
    return Sub(other);
}
Quaternion Quaternion::operator*(const Quaternion &other) const {
    return Mutiply(other);
}