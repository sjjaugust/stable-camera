//
// Created by 张哲华 on 20/09/2017.
//

#include "ThreadContext.h"
using namespace threads;

MySemaphore* ThreadContext::klt_semaphore = nullptr;
MySemaphore* ThreadContext::mc_semaphore = nullptr;
MySemaphore* ThreadContext::out_semaphore = nullptr;
MySemaphore* ThreadContext::read_semaphore= nullptr;

vector<Point2i> ThreadContext::kltList;
list<list<vector<Point2f>>> ThreadContext::trj;
queue<Vec<double, 3>> ThreadContext::rTheta;
Mat ThreadContext::frameVec[ThreadContext::BUFFERSIZE];
Mat ThreadContext::stableTransformVec[ThreadContext::BUFFERSIZE];
Mat ThreadContext::stableRVec[ThreadContext::BUFFERSIZE];
void ThreadContext::Init() {
    Release();
    klt_semaphore = new MySemaphore(0);
    mc_semaphore = new MySemaphore(0);
    out_semaphore = new MySemaphore(0);
    read_semaphore = new MySemaphore(BUFFERSIZE);
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
    kltList.clear();
}