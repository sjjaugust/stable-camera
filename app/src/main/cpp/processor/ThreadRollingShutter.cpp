//
// Created by ShiJJ on 2020/2/27.
//
#include "ThreadRollingShutter.h"
#include <android/log.h>
using namespace cv;
using namespace std;
using namespace threads;

ThreadRollingShutter::~ThreadRollingShutter() {
    work_thread.join();
}
void ThreadRollingShutter::start() {
    work_thread = thread(&ThreadRollingShutter::worker, this);
}
void ThreadRollingShutter::worker() {
    pthread_setname_np(pthread_self(), TAG);
    while (true){
        ThreadContext::rs_semaphore_->Wait();
        if(buffer_index_ < 0){
           ThreadContext::out_semaphore->Signal();
            break;
        }
        __android_log_print(ANDROID_LOG_DEBUG, "ThreadRollingShutter", "i am here");
        getRollingShutterR();
        buffer_index_ = (buffer_index_ + 1) % ThreadContext::BUFFERSIZE;
        ThreadContext::out_semaphore->Signal();
    }
}
void ThreadRollingShutter::getRollingShutterR(){

        Mat gyroInfo = ThreadContext::rs_out_theta_[buffer_index_];
        int gyroInfoRows = gyroInfo.rows;
        double timeStart = gyroInfo.at<double>(0,0);
        double timeEnd = gyroInfo.at<double>(gyroInfoRows-1,0);
//        __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter","RsMat timeEnd:%f", timeEnd);
        vector<double> gyroInfoX, gyroInfoY, gyroInfoZ, gyroInfoOrgTime;
        for(int j = 0; j < gyroInfoRows; j++){
            gyroInfoOrgTime.push_back(gyroInfo.at<double>(j,0));
            gyroInfoX.push_back(gyroInfo.at<double>(j,1));
            gyroInfoY.push_back(gyroInfo.at<double>(j,2));
            gyroInfoZ.push_back(gyroInfo.at<double>(j,3));
        }
        vector<double> timeStampInFrame = getTimeStampInFrame(timeStart, timeEnd, ThreadContext::KRsStripNum_);
        vector<double> gyroInfoInFrameX, gyroInfoInFrameY, gyroInfoInFrameZ;
        gyroInfoInFrameX = interpolation(timeStampInFrame, gyroInfoOrgTime, gyroInfoX);
        gyroInfoInFrameY = interpolation(timeStampInFrame, gyroInfoOrgTime, gyroInfoY);
        gyroInfoInFrameZ = interpolation(timeStampInFrame, gyroInfoOrgTime, gyroInfoZ);
        Mat *rsOutTheta = new Mat[10];
        getMatInFrame(rsOutTheta, gyroInfoInFrameX, gyroInfoInFrameY, gyroInfoInFrameZ);
        gaussSmooth(rsOutTheta);
        for(int j = 0; j < ThreadContext::KRsStripNum_; j++){
            ThreadContext::rs_Mat_[buffer_index_][j] = *(rsOutTheta+j);
        }
        delete[] rsOutTheta;
}
vector<double> ThreadRollingShutter::getTimeStampInFrame(double timestart, double timeend,
                                                         int num) {
    vector<double> timeStampInFrame;
    double space = (timeend-timestart)/(num-1);
    for(int i = 0; i < num; i++){
        double temp = timestart+i*space;
        timeStampInFrame.push_back(temp);
    }
    return timeStampInFrame;
}
vector<double> ThreadRollingShutter::interpolation (vector<double> x0, vector<double> x, vector<double> y){
    vector<double> out;
    for(auto xx0 : x0){
        double yy0 = 0;
        for(int i = 1; i < x.size(); i++){
//            __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter", "%dRsMat xx0:%f, x[i]:%f", i, xx0, x[i]);
            if(xx0>x[i]) continue;
            else if(xx0==x[i]){
                yy0 = y[i];
                break;
            }
            if(i!=0){
                double x1 = x[i-1];
                double y1 = y[i-1];
                double x2 = x[i];
                double y2 = y[i];
                double k = (y2-y1)/(x2-x1);
                double b = y1-k*x1;
                yy0 = k*xx0+b;
                break;
            } else{
                __android_log_print(ANDROID_LOG_ERROR, ThreadRollingShutter::TAG, "xx0 is out of range");

            }
        }
        out.push_back(yy0);
    }
    return out;
}
void ThreadRollingShutter::getMatInFrame(Mat *rsOutTheta, vector<double> gyroInfoInFrameX, vector<double> gyroInfoInFrameY,
                   vector<double> gyroInfoInFrameZ){
    Mat e = Mat::eye(3, 3, CV_64F);
    Mat orgMat(3, 3, CV_64F);
    Mat skewOrgMat(3, 3, CV_64F);
//    计算第一条旋转矩阵
    double thOrg = gyroInfoInFrameX[0]*gyroInfoInFrameX[0]+
                   gyroInfoInFrameY[0]*gyroInfoInFrameY[0]+
                   gyroInfoInFrameZ[0]*gyroInfoInFrameZ[0];
    thOrg = sqrt(thOrg);

    if(thOrg == 0){
        thOrg = 1;
    }
    skewOrgMat.at<double>(0,0) = 0;
    skewOrgMat.at<double>(0,1) = -gyroInfoInFrameZ[0]/thOrg;
    skewOrgMat.at<double>(0,2) = gyroInfoInFrameY[0]/thOrg;
    skewOrgMat.at<double>(1,0) = gyroInfoInFrameZ[0]/thOrg;
    skewOrgMat.at<double>(1,1) = 0;
    skewOrgMat.at<double>(1,2) = -gyroInfoInFrameX[0]/thOrg;
    skewOrgMat.at<double>(2,0) = -gyroInfoInFrameY[0]/thOrg;
    skewOrgMat.at<double>(2,1) = gyroInfoInFrameX[0]/thOrg;
    skewOrgMat.at<double>(2,2) = 0;
    orgMat = e+sin(thOrg)*skewOrgMat.t()+(1-cos(thOrg))*(skewOrgMat*skewOrgMat).t();

    for(int i = 0; i < gyroInfoInFrameX.size(); i++){
        Mat cvMat(3, 3, CV_64F);
        Mat skewMat(3, 3, CV_64F);
        double th = gyroInfoInFrameX[i]*gyroInfoInFrameX[i]+
                gyroInfoInFrameY[i]*gyroInfoInFrameY[i]+
                gyroInfoInFrameZ[i]*gyroInfoInFrameZ[i];
        th = sqrt(th);
        int eq = 0;
        if(th == 0){
            eq = 1;
        }
        th += eq;
        gyroInfoInFrameX[i] /= th;
        gyroInfoInFrameY[i] /= th;
        gyroInfoInFrameZ[i] /= th;

        skewMat.at<double>(0,0) = 0;
        skewMat.at<double>(0,1) = -gyroInfoInFrameZ[i];
        skewMat.at<double>(0,2) = gyroInfoInFrameY[i];
        skewMat.at<double>(1,0) = gyroInfoInFrameZ[i];
        skewMat.at<double>(1,1) = 0;
        skewMat.at<double>(1,2) = -gyroInfoInFrameX[i];
        skewMat.at<double>(2,0) = -gyroInfoInFrameY[i];
        skewMat.at<double>(2,1) = gyroInfoInFrameX[i];
        skewMat.at<double>(2,2) = 0;
        cvMat = e+sin(th)*skewMat.t()+(1-cos(th))*(skewMat*skewMat).t();

        cv::Mat a1 = (cv::Mat_<double>(3, 3) << 0.0,1.0,0.0, -1.0,0.0,0.0, 0.0 ,0.0 ,1.0);
        cv::Mat a2 = (cv::Mat_<double>(3, 3) << 1,0,0, 0,-1,0, 0 ,0 ,-1);
        cv::Mat A=a1*a2;
        cv::Mat hom = cv::Mat::eye(cv::Size(3,3),CV_64F);
//        Mat outMat = inmat*A.t()*cvMat*orgMat.t()*hom.t()*A*inmat.inv();
        Mat outMat = inmat*A.t()*orgMat*cvMat.t()*hom.t()*A*inmat.inv();

        outMat.copyTo(*(rsOutTheta+i));
    }
//    for(int i = 0; i < gyroInfoInFrameX.size(); i++){
//        Mat cvMat(3, 3, CV_64F);
//        Mat skewMat(3, 3, CV_64F);
//        double angle_diff_x = -(gyroInfoInFrameX[i]-gyroInfoInFrameX[0]);
//        double angle_diff_y = -(gyroInfoInFrameY[i]-gyroInfoInFrameY[0]);
//        double angle_diff_z = -(gyroInfoInFrameZ[i]-gyroInfoInFrameZ[0]);
//        double th = angle_diff_x * angle_diff_x +
//                angle_diff_y * angle_diff_y +
//                angle_diff_z * angle_diff_z;
//        th = sqrt(th);
//        int eq = 0;
//        if(th == 0){
//            eq = 1;
//        }
//        th += eq;
//        gyroInfoInFrameX[i] /= th;
//        gyroInfoInFrameY[i] /= th;
//        gyroInfoInFrameZ[i] /= th;
//
//        skewMat.at<double>(0,0) = 0;
//        skewMat.at<double>(0,1) = -angle_diff_z;
//        skewMat.at<double>(0,2) = angle_diff_y;
//        skewMat.at<double>(1,0) = angle_diff_z;
//        skewMat.at<double>(1,1) = 0;
//        skewMat.at<double>(1,2) = -angle_diff_x;
//        skewMat.at<double>(2,0) = -angle_diff_y;
//        skewMat.at<double>(2,1) = angle_diff_x;
//        skewMat.at<double>(2,2) = 0;
//        __android_log_print(ANDROID_LOG_DEBUG, "ThreadRollingShutter", "angle_diff:%f, %f ,%f", angle_diff_x, angle_diff_y, angle_diff_z);
//        cvMat = e+sin(th)*skewMat.t()+(1-cos(th))*(skewMat*skewMat).t();
//        Mat outMat = inmat*cvMat*inmat.inv();
//        outMat.copyTo(*(rsOutTheta+i));
//    }

}

void ThreadRollingShutter::gaussSmooth(Mat *rsOutTheta){
    Mat *temp = new Mat[10];
    for(int i = 0 ; i < 10; i++){
        (rsOutTheta+i)->copyTo(*(temp+i));
    }

    for(int i = 0; i < 10; i++){
        int count = 1;
        rsOutTheta[i] = temp[i]*ThreadContext::guass_weight_[5];
        while (count<=5){
            if(i-count>=0&&i+count<=9){
                rsOutTheta[i] = rsOutTheta[i]+temp[i+count]*ThreadContext::guass_weight_[5+count]
                        +temp[i-count]*ThreadContext::guass_weight_[5-count];
            } else if(i-count<0){
                rsOutTheta[i] = rsOutTheta[i]+temp[i+count]*ThreadContext::guass_weight_[5+count]
                        +temp[abs(i-count)-1]*ThreadContext::guass_weight_[5-count];
            } else if(i+count>9){
                rsOutTheta[i] = rsOutTheta[i]+temp[i-count]*ThreadContext::guass_weight_[5-count]
                        +temp[9-(i+count-9-1)]*ThreadContext::guass_weight_[5+count];
            }
            count++;
        }
    }
}
Mat ThreadRollingShutter::constantMulMat(Mat &src, double num){
    int width = src.cols;
    int height = src.rows;
    Mat dst(height, width, src.type());
    src.copyTo(dst);
    for(int i = 0;i<height; i++){
        for(int j = 0; j<width; j++){
            dst.at<double>(i,j) = dst.at<double>(i, j)*num;
        }
    }
    return dst;
}

void ThreadRollingShutter::showMat(Mat cvMat) {
    for(int i = 0; i < cvMat.rows; i++){
        for(int j = 0; j < cvMat.cols; j++){
            __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter", "(%d, %d):showMat:%f", i, j, cvMat.at<double>(i, j));
        }
    }
}