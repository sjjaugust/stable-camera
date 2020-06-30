//
// Created by ShiJJ on 2020/6/28.
//

#ifndef BSPLINE_BSPLINE_H
#define BSPLINE_BSPLINE_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>

class Bspline {
private:
    std::vector<cv::Point2d> data_point_;
    std::vector<cv::Point2d> control_point_;
    std::vector<double> u_;
    std::vector<double> distance_;//存放数据点之间距离
    double sum_distance_ = 0;//数据点距离之和
    std::vector<double> delta_;
    int d_num_ = 0;//数据点个数
    int u_num_ = 0;//节点个数
    int c_num_ = 0;//控制点个数
    std::vector<cv::Point2d> curve_point_;

    void init();//push所有数据点后进行初始化
    double calDistance(cv::Point2d p1, cv::Point2d p2);
    double base(int i, int k, double u);
public:
    void push(cv::Point2d point);
    void calControlPoint();
    void genCurve();
    void show();
    cv::Point2d genInterpolationPoint(double ratio);

};


#endif //BSPLINE_BSPLINE_H
