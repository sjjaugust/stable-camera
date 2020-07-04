//
// Created by ShiJJ on 2020/2/27.
//
#include "ThreadRollingShutter.h"
#include <android/log.h>
using namespace cv;
using namespace std;
using namespace threads;
static Mat RR2stableVec = (cv::Mat_<double>(3, 3)<<0.0, 1.0, 0.0, -1.0, 0.0, 1080.0, 0.0, 0.0, 1.0);
static Mat stableVec2RR = (cv::Mat_<double>(3, 3)<<0.0, -1.0, 1080.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
bool ThreadRollingShutter::is_stable_ = false;
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
        double timeEnd = gyroInfo.at<double>(gyroInfoRows-5, 0);
//        __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter","RsMat timeEnd:%f", timeEnd);
        vector<double> gyroInfoX, gyroInfoY, gyroInfoZ, gyroInfoOrgTime;
        for(int j = 0; j < gyroInfoRows-4; j++){
            gyroInfoOrgTime.push_back(gyroInfo.at<double>(j,0));
            gyroInfoX.push_back(gyroInfo.at<double>(j,1));
            gyroInfoY.push_back(gyroInfo.at<double>(j,2));
            gyroInfoZ.push_back(gyroInfo.at<double>(j,3));
        }

        vector<double> timeStampInFrame = getTimeStampInFrame(timeStart, timeEnd, ThreadContext::KRsStripNum_);
        LOGI("timestampinframe:[%f, %f, %f, %f, %f, %f, %f, %f, %f, %f]", timeStampInFrame[0], timeStampInFrame[1], timeStampInFrame[2],
             timeStampInFrame[3], timeStampInFrame[4], timeStampInFrame[5],
             timeStampInFrame[6], timeStampInFrame[7], timeStampInFrame[8], timeStampInFrame[9]);
        vector<double> gyroInfoInFrameX, gyroInfoInFrameY, gyroInfoInFrameZ;
        gyroInfoInFrameX = interpolation(timeStampInFrame, gyroInfoOrgTime, gyroInfoX);
        gyroInfoInFrameY = interpolation(timeStampInFrame, gyroInfoOrgTime, gyroInfoY);
        gyroInfoInFrameZ = interpolation(timeStampInFrame, gyroInfoOrgTime, gyroInfoZ);
        Mat *rsOutTheta = new Mat[10];
        if(!is_stable_){
            getMatInFrame(rsOutTheta, gyroInfoInFrameX, gyroInfoInFrameY, gyroInfoInFrameZ);
            gaussSmooth(rsOutTheta);
            for(int j = 0; j < ThreadContext::KRsStripNum_; j++){
                ThreadContext::rs_Mat_[buffer_index_][j] = *(rsOutTheta+j);
            }
        } else{
            for(int j = 0; j < ThreadContext::KRsStripNum_; j++){
                ThreadContext::rs_Mat_[buffer_index_][j] = cv::Mat::eye(3, 3, CV_64F);
            }
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

//    gyroInfoInFrameX[0] = 0;
    gyroInfoInFrameY[0] = gyroInfoInFrameY[0];
    gyroInfoInFrameZ[0] = 0;
//    计算第一条旋转矩阵
    Quaternion q0 = Quaternion::EulerToQuaternion(gyroInfoInFrameX[0], gyroInfoInFrameY[0], gyroInfoInFrameZ[0]);
    orgMat = Quaternion::QuaternionToR(q0);

    angle_.clear();
    angle_.resize(3);
    for(int i = 0; i < 3; i++){
        angle_[i] = 0;
    }
    std::vector<double> temp_x = gyroInfoInFrameX;
    std::vector<double> temp_y = gyroInfoInFrameY;
    std::vector<double> temp_z = gyroInfoInFrameZ;
    for(int i = 0; i < gyroInfoInFrameX.size(); i++){
        if(i != 0){
//            gyroInfoInFrameX[i] = 0;
            gyroInfoInFrameY[i] = gyroInfoInFrameY[i];
            gyroInfoInFrameZ[i] = 0;
        }

        angle_[0] += abs(temp_x[i]-temp_x[0]);
        angle_[1] += abs(temp_y[i]-temp_y[0]);
        angle_[2] += abs(temp_z[i]-temp_z[0]);
        Mat cvMat(3, 3, CV_64F);
        Quaternion q = Quaternion::EulerToQuaternion(gyroInfoInFrameX[i], gyroInfoInFrameY[i], gyroInfoInFrameZ[i]);
        Quaternion convert = Quaternion::Q1ToQ2(q, q0);
        cvMat = Quaternion::QuaternionToR(convert);
        Mat outMat = inmat * cvMat * inmat.inv();

        outMat.copyTo(*(rsOutTheta+i));
    }

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
void ThreadRollingShutter::getStableStatus(bool is_stable) {
    is_stable_ = is_stable;
}