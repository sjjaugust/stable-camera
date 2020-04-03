//
// Created by 张哲华 on 20/09/2017.
//

#ifndef VIDEOSTABLE_THREADCONTEXT_H
#define VIDEOSTABLE_THREADCONTEXT_H

#include "MySemaphore.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <list>

using namespace std;
using namespace cv;
namespace threads {
    class ThreadContext {
    public:
        const static int SEGSIZE = 5;
        const static int BUFFERSIZE = SEGSIZE * 4;
        const static int MIN_DISTANCE = 20;
        constexpr const static float TRANSLATE_AMPLITUDE = 0.4f;
        constexpr const static float ROTATE_AMPLITUDE = 0.02f;
        const static int DOWNSAMPLE_SCALE = 4;

        static MySemaphore* read_semaphore;
        static MySemaphore* klt_semaphore;
        static MySemaphore* mc_semaphore;
        static MySemaphore* out_semaphore;

        static vector<Point2i> kltList;
        static list<list<vector<Point2f>>> trj;
        static queue<Vec<double, 3>> rTheta;
        static Mat frameVec[BUFFERSIZE];
        static Mat stableTransformVec[BUFFERSIZE];
        static Mat stableRVec[BUFFERSIZE];

        static void Init();
        static void Release();
    };
}


#endif //VIDEOSTABLE_THREADCONTEXT_H
