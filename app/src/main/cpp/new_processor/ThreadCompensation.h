//
// Created by ShiJJ on 2020/4/18.
//

#ifndef STABLE_CAMERA_THREADCOMPENSATION_H
#define STABLE_CAMERA_THREADCOMPENSATION_H

#include <thread>
#include "ThreadContext.h"
#include "Filiter.h"
#include <stdio.h>
#include "DigitalFilter.h"

class ThreadCompensation {
private:
    std::thread work_thread_;
    int cm_index_ = 0;
    int out_index_ = 0;
    Filiter filiter;
    Filiter filiter1;
    Filiter filiter2;
    std::queue<Quaternion> q_cache_;
    const double crop_ratio_ = 0.9;
    cv::Size frame_size_;
    bool is_first = true;
    Quaternion nn;
    Quaternion last;
    std::queue<std::vector<Quaternion>> rs_q_cache_;
public:
    bool crop_control_flag = true;
private:
    void Work();
    void FrameCompensation();
    bool CropControl(double crop_ratio, const cv::Size& size, cv::Mat& mat);
    bool IsInside(cv::Mat crop_vertex, cv::Mat new_vertex);
    void RollingShutter(int cm_index);
    std::vector<double> GetTimeStampInFrame(double timestart, double timeend,
                                            int num);
    void WriteDataToFile(std::FILE* file_old, const Quaternion& old_q, std::FILE* file_new, const Quaternion& new_q, int frame);
public:
    void Start();
    ~ThreadCompensation();
////*****************数字处理********************////
private:
    int cm_las_index_ = 0;
    int cm_cur_index_ = 1;
    int ex_index_ = 0;
    cv::Mat last_gray_;
    cv::Mat cur_gray_;
    std::vector<cv::Point2f> last_features_,cur_features_;
    std::vector<cv::Point2f> last_features_tmp_,cur_features_tmp_;
    std::vector<uchar> status_,status_choose_;
    cv::Vec<double, 3> last_rot_;
    bool is_first_use_rtheta_ = true;
    cv::Mat h_scale_;
    bool with_roll_ = false;
    DigitalFilter digital_filter_;
    std::queue<cv::Mat> aff_queue_;
private:
    double PointDistance(const cv::Point2f& p1,  const cv::Point2f& p2);
    bool OutOfImg(const cv::Point2f& point, const cv::Size& size);
    cv::Mat ComputeAffine();
    void DetectFeature();
    void TrackFeature();
    bool StableCount(double e);
    double LineDistance(cv::Point2f p0, cv::Point2f p1, cv::Point2f p2);
    cv::Mat CalculHomo(int niter);
    cv::Mat MoveAndScale();

public:

};


#endif //STABLE_CAMERA_THREADCOMPENSATION_H
