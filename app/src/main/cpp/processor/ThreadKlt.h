//
// Created by 张哲华 on 19/09/2017.
//

#ifndef VIDEOSTABLE_KLTTHREAD_H
#define VIDEOSTABLE_KLTTHREAD_H

#include <opencv2/opencv.hpp>
#include <thread>
#include "ThreadContext.h"
#include "MySemaphore.h"
#include "TimeRecorder.h"

using namespace std;
using namespace cv;

namespace threads {
    // klt thread is triggered once created
    class ThreadKlt {
    private:
        const char* TAG = "KltThread";
        void worker();
        thread worker_thread_;
        bool outOfImg(const Point2f &point, const Size &size);
        bool isTheSame(const Point2f &point, const Point2f &pt);
        void downSampling( Mat &img );
        void constructTrajectory();
    public:
        Size videoSize;
        void start();
        virtual ~ThreadKlt();
    };
}


#endif //VIDEOSTABLE_KLTTHREAD_H
