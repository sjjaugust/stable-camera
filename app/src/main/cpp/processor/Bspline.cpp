//
// Created by ShiJJ on 2020/6/28.
//

#include "Bspline.h"
void Bspline::init() {
    d_num_ = data_point_.size();
    c_num_ = d_num_ + 2;
    u_num_ = c_num_ + 3 + 1;
    data_point_.resize(d_num_);
    u_.resize(u_num_);
    if(d_num_ < 2){
        std::cout << "Too little data!!, the amount of data is  " << d_num_ << std::endl;
        return;
    }
    //计算数据点之间的距离,distance_[0]代表第一个点到第二个点之间的距离
    distance_.resize(d_num_ - 1);
    for(int i = 0; i < d_num_ - 1; i++){
        distance_[i] = calDistance(data_point_[i], data_point_[i+1]);
        sum_distance_ += distance_[i];
    }
    //初始化节点
    for(int i = 0; i < 4; i++){
        u_[i] = 0;
    }
    for(int i = u_num_ - 4; i <= u_num_ - 1; i++){
        u_[i] = 1;
    }
    int k = 1;
    for(int i = 4; i < u_num_ - 4; i++){
        double temp = 0;
        int l = k;
        while (l){
            temp += distance_[l-1];
            l--;
        }
        k++;
        temp /= sum_distance_;
        u_[i] = temp;
    }
    std::cout << "u_: [ ";
    for(int i = 0; i < u_num_; i++){
        std::cout << u_[i] << " ";
    }
    std::cout << "]" << std::endl;
    //初始化delta
    delta_.resize(u_num_ - 1);
    for(int i = 0; i < u_num_ - 1; i++){
        delta_[i] = u_[i+1] - u_[i];
    }
}

void Bspline::calControlPoint() {
    init();
    double a[d_num_], b[d_num_], c[d_num_], d[d_num_], e[d_num_], f[d_num_];
    a[0]=1-(delta_[3]*delta_[4])/pow((delta_[3]+delta_[4]),2);
    b[0]=(delta_[3]/(delta_[3]+delta_[4]))*(delta_[4]/(delta_[3]+delta_[4])-delta_[3]/(delta_[3]+delta_[4]+delta_[5]));
    c[0]=(pow((delta_[3]),2))/((delta_[3]+delta_[4])*(delta_[3]+delta_[4]+delta_[5]));
    e[0]=(1.0/3)*(data_point_[0].x+2*data_point_[1].x);  //X轴坐标
    f[0]=(1.0/3)*(data_point_[0].y+2*data_point_[1].y);  //Y轴坐标

    a[d_num_-1]=-(pow((delta_[d_num_+1]),2))
            /((delta_[d_num_]+delta_[d_num_+1])*(delta_[d_num_]+delta_[d_num_]+delta_[d_num_+1]));
    b[d_num_-1]=(delta_[d_num_+1]/(delta_[d_num_]+delta_[d_num_+1]))*
            (delta_[d_num_+1]/(delta_[d_num_]+delta_[d_num_]+delta_[d_num_+1])-delta_[d_num_]/(delta_[d_num_]+delta_[d_num_+1]));
    c[d_num_-1]=delta_[d_num_]*delta_[d_num_+1]/pow((delta_[d_num_]+delta_[d_num_+1]),2)-1;
    e[d_num_-1]=-(1.0/3)*(data_point_[d_num_-1].x+2*data_point_[d_num_-2].x);
    f[d_num_-1]=-(1.0/3)*(data_point_[d_num_-1].y+2*data_point_[d_num_-2].y);

    for(int i = 1; i < d_num_ - 1; i++){
        a[i]=(pow((delta_[i+3]),2))/(delta_[i+1]+delta_[i+2]+delta_[i+3]);
        b[i]=delta_[i+3]*(delta_[i+1]+delta_[i+2])
                /(delta_[i+1]+delta_[i+2]+delta_[i+3])+delta_[i+2]*(delta_[i+3]+delta_[i+4])
                /(delta_[i+2]+delta_[i+3]+delta_[i+4]);
        c[i]=pow((delta_[i+2]),2)/(delta_[i+2]+delta_[i+3]+delta_[i+4]);
        e[i]=(delta_[i+2]+delta_[i+3])*data_point_[i].x;
        f[i]=(delta_[i+2]+delta_[i+3])*data_point_[i].y;
    }
    cv::Mat mat = cv::Mat::zeros(d_num_, d_num_, CV_64F);
    mat.at<double>(0, 0) = a[0];
    mat.at<double>(0, 1) = b[0];
    mat.at<double>(0, 2) = c[0];
    for(int i=1;i<d_num_ - 1;i++)
    {
        mat.at<double>(i, i-1) = a[i];
        mat.at<double>(i, i) = b[i];
        mat.at<double>(i, i+1) = c[i];
    }
    mat.at<double>(d_num_ - 1, d_num_ - 3)=a[d_num_ - 1];
    mat.at<double>(d_num_ - 1, d_num_ - 2)=b[d_num_ - 1];
    mat.at<double>(d_num_ - 1, d_num_ - 1)=c[d_num_ - 1];
    std::cout << "mat" << mat << std::endl;
    cv::Mat mat_inv = mat.inv();
    //计算控制点
    double sum_x[d_num_], sum_y[d_num_];
    std::memset(sum_x, 0, sizeof(sum_x));
    std::memset(sum_y, 0, sizeof(sum_y));
    control_point_.push_back(data_point_[0]);
    for(int i = 0; i < d_num_; i++){
        for(int j = 0; j < d_num_; j++){
            sum_x[i] = sum_x[i] + mat_inv.at<double>(i, j) * e[j];
            sum_y[i] = sum_y[i] + mat_inv.at<double>(i, j) * f[j];
        }
        control_point_.emplace_back(cv::Point2d(sum_x[i], sum_y[i]));
    }
    control_point_.push_back(data_point_[d_num_ - 1]);
    std::cout << "control points:[ ";
    for(auto i : control_point_){
        std::cout << i << ", ";
    }
    std::cout << " ]" << std::endl;
}

void Bspline::genCurve() {
    curve_point_.clear();
    int i = 3;
    for(double u = 0; u < 1; u += 0.001){
        cv::Point2d temp(0, 0);
        if(u > u_[i+1]){
            i++;
        }
        for(int k = 0; k < 4; k++){
            cv::Point2d p = control_point_[i - k];
            p *= base(i - k, 3, u);
            temp += p;
        }
        curve_point_.push_back(temp);
    }
}

cv::Point2d Bspline::genInterpolationPoint(double ratio) {
    int i;
    for(i = 3; i <= u_.size() - 3; i++){
        if(ratio < u_[i]){
            i--;
            break;
        }
    }
    cv::Point2d temp(0, 0);
    for(int k = 0; k < 4; k++){
        cv::Point2d p = control_point_[i - k];
        p *= base(i - k, 3, ratio);
        temp += p;
    }
    return temp;
}

void Bspline::show() {
    cv::Mat plane(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255));
    std::cout <<curve_point_.size()<<std::endl;
    for(auto p : curve_point_){
        cv::circle(plane, p, 2, cv::Scalar(0, 0, 255), 1);
    }
    for(auto p : data_point_){
        cv::circle(plane, p, 2, cv::Scalar(255, 0, 0), 3);
    }
    cv::circle(plane, genInterpolationPoint(0.5), 3, cv::Scalar(255, 0, 0), 5);
//    for(auto p : control_point_){
//        cv::circle(plane, p, 2, cv::Scalar(0, 255, 0), 1);
//    }
    cv::imshow("curve", plane);
    cv::waitKey(0);
}

double Bspline::base(int i, int k, double u) {
    if(k == 0){
        if(u >= u_[i] && u < u_[i+1]){
            return 1;
        } else {
            return 0;
        }
    } else {
        double a1 = (u - u_[i]);
        double b1 = (u_[i+k] - u_[i]);
        double a2 = (u_[i+k+1] - u);
        double b2 = (u_[i+k+1] - u_[i+1]);
        if(b1 < 0.001){
            a1 = 0;
            b1 = 1;
        }
        if(b2 < 0.001){
            a2 = 0;
            b2 = 1;
        }
        return (a1 * base(i, k-1, u)) / b1 + (a2 * base(i+1, k-1, u)) / b2;

    }

}

void Bspline::push(cv::Point2d point) {
    data_point_.push_back(point);
}

double Bspline::calDistance(cv::Point2d p1, cv::Point2d p2) {
    return sqrt(pow((p2.y - p1.y), 2) + pow((p2.x - p1.x), 2));
}