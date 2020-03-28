//
// Created by 张哲华 on 20/09/2017.
//

#include "ThreadContext.h"
using namespace threads;

MySemaphore* ThreadContext::klt_semaphore = nullptr;//光流信号量
MySemaphore* ThreadContext::mc_semaphore = nullptr;//运动估计信号量
MySemaphore* ThreadContext::out_semaphore = nullptr;//输出信号量
MySemaphore* ThreadContext::read_semaphore= nullptr;//读入信号量

vector<Point2i> ThreadContext::kltList;
vector<Point2i> ThreadContext::motionCompList;
vector<Point2i> ThreadContext::outputList;
list<list<vector<Point2f>>> ThreadContext::trj;
Mat ThreadContext::frameVec[ThreadContext::BUFFERSIZE];//帧向量
Mat ThreadContext::stableTransformVec[ThreadContext::BUFFERSIZE];
Mat ThreadContext::stableRVec[ThreadContext::BUFFERSIZE];
//果冻效应相关
Mat ThreadContext::rsOutTheta[ThreadContext::BUFFERSIZE];
vector<Point2i> ThreadContext::rsList;
MySemaphore* ThreadContext::rs_semaphore = nullptr;
Mat ThreadContext::rsMat[ThreadContext::BUFFERSIZE][ThreadContext::rsStripNum];
double ThreadContext::gaussWeight[];
void ThreadContext::createGaussWeight(double *gaussWeight) {
    int num = 0;
    for(int i = -5; i < 6; i++){
        double temp = -(i*i)/2.0;
        double xishu = 1.0/sqrt(2.0*pi);
        *(gaussWeight+num) = xishu*exp(temp);
        num++;
//        __android_log_print(ANDROID_LOG_ERROR, "ThreadContext", "%fgaussWeight:%f",temp , *(gaussWeight+num-1));
    }
}

void ThreadContext::Init() {
    Release();
    klt_semaphore = new MySemaphore(0);
    mc_semaphore = new MySemaphore(0);
    out_semaphore = new MySemaphore(0);
    rs_semaphore = new MySemaphore(0);
    read_semaphore = new MySemaphore(4);
    createGaussWeight(gaussWeight);
}

void ThreadContext::Release() {
    if (read_semaphore != nullptr) {
        delete read_semaphore;
        read_semaphore = nullptr;
    }
    if (klt_semaphore != nullptr) {
        delete klt_semaphore;
        klt_semaphore = nullptr;
    }
    if (mc_semaphore != nullptr) {
        delete mc_semaphore;
        mc_semaphore = nullptr;
    }
    if (out_semaphore != nullptr) {
        delete out_semaphore;
        out_semaphore = nullptr;
    }
    if(rs_semaphore != nullptr){
        delete rs_semaphore;
        rs_semaphore = nullptr;
    }
    kltList.clear();
    motionCompList.clear();
    outputList.clear();
    rsList.clear();
}