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
        ThreadContext::rs_semaphore->Wait();
        if(ThreadContext::rsList[0].y<ThreadContext::SEGSIZE){
            ThreadContext::out_semaphore->Signal();
            break;
        }
        getRollingShutterR();

        //确保每次都是最上面
        ThreadContext::rsList.erase(ThreadContext::rsList.begin());
        ThreadContext::out_semaphore->Signal();
    }
}
void ThreadRollingShutter::getRollingShutterR(){
    int start = ThreadContext::rsList[0].x;
    int length = ThreadContext::rsList[0].y;
    for(int i = 0; i < length; i++){
        int index = (start+i)%ThreadContext::BUFFERSIZE;

        Mat gyroInfo = ThreadContext::rsOutTheta[index];
        double timeStart = gyroInfo.at<double>(0,0);
        double timeEnd = gyroInfo.at<double>(2,0);
//        __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter","RsMat timeEnd:%f", timeEnd);
        vector<double> gyroInfoX, gyroInfoY, gyroInfoZ, gyroInfoOrgTime;
        for(int j = 0; j < 3; j++){
            gyroInfoOrgTime.push_back(gyroInfo.at<double>(j,0));
            gyroInfoX.push_back(gyroInfo.at<double>(j,1));
            gyroInfoY.push_back(gyroInfo.at<double>(j,2));
            gyroInfoZ.push_back(gyroInfo.at<double>(j,3));
        }
        vector<double> timeStampInFrame = getTimeStampInFrame(timeStart, timeEnd, ThreadContext::rsStripNum);
//        for(int l = 0; l < timeStampInFrame.size(); l++){
//            __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter", "%dRsMat:%f", l, timeStampInFrame[l]);
//        }
//        for(int l = 0; l < gyroInfoX.size(); l++){
//            __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter", "%dRsMat:%f", l, gyroInfoX[l]);
//        }

        vector<double> gyroInfoInFrameX, gyroInfoInFrameY, gyroInfoInFrameZ;
        gyroInfoInFrameX = interpolation(timeStampInFrame, gyroInfoOrgTime, gyroInfoX);
        gyroInfoInFrameY = interpolation(timeStampInFrame, gyroInfoOrgTime, gyroInfoY);
        gyroInfoInFrameZ = interpolation(timeStampInFrame, gyroInfoOrgTime, gyroInfoZ);
//        for(int o = 0; o < gyroInfoInFrameX.size(); o++){
//            gyroInfoInFrameX[o] *= 100.0;
//            gyroInfoInFrameY[o] *= 100.0;
//            gyroInfoInFrameZ[o] *= 100.0;
//        }
        for(int l = 0; l < gyroInfoX.size(); l++){
            __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter", "%dRsMat timeinframe:%f", l, gyroInfoX[l]);
        }
//        for(int l = 0; l < gyroInfoOrgTime.size(); l++){
//            __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter", "%dRsMat gyroInfoOrgTime:%f", l, gyroInfoOrgTime[l]);
//        }
//        for(int l = 0; l < gyroInfoInFrameX.size(); l++){
//            __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter", "%dRsMat gyroInfoInFrameX:%f", l, gyroInfoInFrameX[l]-gyroInfoInFrameX[0]);
//        }
        Mat *rsOutTheta = new Mat[10];
        getMatInFrame(rsOutTheta, gyroInfoInFrameX, gyroInfoInFrameY, gyroInfoInFrameZ);
        gaussSmooth(rsOutTheta);
        for(int j = 0; j < ThreadContext::rsStripNum; j++){
            ThreadContext::rsMat[index][j] = *(rsOutTheta+j);
//            __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter", "index:%d,RsMat:%f", index, (rsOutTheta+j)->at<double>(0,0));
        }
        delete[] rsOutTheta;
    }
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
    Mat orgMat(3, 3, CV_64F);
    Mat skewOrgMat(3, 3, CV_64F);
    Mat e = Mat::eye(3, 3, CV_64F);
    double thOrg = gyroInfoInFrameX[0]*gyroInfoInFrameX[0]+
                   gyroInfoInFrameY[0]*gyroInfoInFrameY[0]+
                   gyroInfoInFrameZ[0]*gyroInfoInFrameZ[0];
    thOrg = sqrt(thOrg);
    __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter", "thOrg:%f", thOrg);
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
//    for(int i = 0; i < 3; i++){
//        for(int j = 0; j < 3; j++){
//            __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter", "RsMat:%f", orgMat.at<double>(i,j));
//        }
//    }
//测试啊啊啊
    for(int i = 1; i < gyroInfoInFrameX.size(); i++){
        double tempx = gyroInfoInFrameX[i]-gyroInfoInFrameX[0];
        double tempy = gyroInfoInFrameY[i]-gyroInfoInFrameY[0];
        double tempz = gyroInfoInFrameZ[i]-gyroInfoInFrameZ[0];
        gyroInfoInFrameX[i]-=tempx*10;
        gyroInfoInFrameY[i]-=tempy*10;
        gyroInfoInFrameZ[i]-=tempz*10;
        __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter", "%dtempxxxx%f", i, tempy);
    }
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
        __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter", "RsMatxxxxxxxxxxx:%f", 1-cos(th));
//        for(int l = 0; l < 3; l++){
//            for(int j = 0; j < 3; j++){
//                __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter", "RsMat:%f", orgMat.at<double>(l,j));
//            }
//        }
//        cvMat = cvMat*orgMat ;
//        Mat a1 = (cv::Mat_<double>(3, 3) << 0.0,1.0,0.0, -1.0,0.0,0.0, 0.0 ,0.0 ,1.0);
//        Mat a2 = (cv::Mat_<double>(3, 3) << 1,0,0, 0,-1,0, 0 ,0 ,-1);
//        Mat A=a1*a2;
//        Mat sta = cvMat.t()*orgMat;
//        Mat temp = A.t()*sta.t()*A;
//        Mat outMat = inmat*temp*inmat.inv();
        Mat outMat = inmat*cvMat*orgMat.inv()*inmat.inv();
//        __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter", "RsMat:%f", orgMat.at<double>(0,0));
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
        rsOutTheta[i] = temp[i]*ThreadContext::gaussWeight[5];
        while (count<=5){
            if(i-count>=0&&i+count<=9){
                rsOutTheta[i] = rsOutTheta[i]+temp[i+count]*ThreadContext::gaussWeight[5+count]
                        +temp[i-count]*ThreadContext::gaussWeight[5-count];
            } else if(i-count<0){
                rsOutTheta[i] = rsOutTheta[i]+temp[i+count]*ThreadContext::gaussWeight[5+count]
                        +temp[abs(i-count)-1]*ThreadContext::gaussWeight[5-count];
            } else if(i+count>9){
                rsOutTheta[i] = rsOutTheta[i]+temp[i-count]*ThreadContext::gaussWeight[5-count]
                        +temp[9-(i+count-9-1)]*ThreadContext::gaussWeight[5+count];
            }
            count++;
        }
    }

//    for(int i = 0; i < 3; i++){
//        for(int j = 0; j <3; j++){
//            __android_log_print(ANDROID_LOG_ERROR, "ThreadRollingShutter", "i:%d,j:%d,position--:%f", i, j, rsOutTheta[position].at<double>(i, j));
//        }
//    }

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