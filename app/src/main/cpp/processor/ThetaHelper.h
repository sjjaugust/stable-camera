//
// Created by 张哲华 on 2019/3/1.
//

#ifndef GRYOSTABLE_THETAHELPER_H
#define GRYOSTABLE_THETAHELPER_H

#include "opencv2/opencv.hpp"
using namespace cv;
using namespace std;
class ThetaHelper {
private:
//    Mat inmat=(cv::Mat_<double>(3, 3)<<1440.0,0.0,540.0, 0.0,1440.0,960.0,0.0,0.0,1.0);
    Mat inmat=(cv::Mat_<double>(3, 3)<<600.0,0.0,616.0, 0.0,600.0,956.0,0.0,0.0,1.0);
    Mat vertex=(cv::Mat_<double>(3, 4)<<0.0,0.0,1080.0,1080.0,0.0,1920.0,1920.0,0.0,1.0,1.0,1.0,1.0);
    Mat cropvertex=(cv::Mat_<double>(3, 4)<<108.0,108.0,972.0, 972.0, 192.0 , 1728.0, 1728.0, 192.0,1.0,1.0,1.0,1.0);
    Mat I=(cv::Mat_<double>(3, 3)<<1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0);


    int c;
    int gyindex;
    int findex;
    int angledex;

    vector<double> roxl;
    vector<double> royl;
    vector<double> rozl;
    vector<double> Timeg;
    vector<double> Timeframe;
    vector<double> oldx;
    vector<double> oldy;
    vector<double> oldz;
    double lastx;
    double lasty;
    double lastz;


    cv::Mat result ;
    cv::Mat RR;

    double px;
    double py;
    double pz;
    double q;
    double rx;
    double ry;
    double rz;
    double kx;
    double ky;
    double kz;
    cv::Vec<double, 3> lasttheta;
    cv::Vec<double, 3> lastt;
    void cropControl(Mat& RR);
    bool isInside(cv::Mat cropvertex, cv::Mat newvertex);

public:
    void init();
    Mat getRR(Mat oldRotation, Mat newRotation);
    cv::Vec<double, 3> getTheta();
    cv::Vec<double, 3> getNewTheta(cv::Vec<double, 3> oldtheta);
    cv::Mat getRotationMat(cv::Vec<double, 3> theta);
    void getR(double timestamp, Mat *matR, bool isCrop);
    void putValue(double timestamp, float x, float y, float z);
};


#endif //GRYOSTABLE_THETAHELPER_H
