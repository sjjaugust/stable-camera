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
        const static int MIN_DISTANCE = 20;//两点相似的最小距离
        constexpr const static float TRANSLATE_AMPLITUDE = 0.4f;
        constexpr const static float ROTATE_AMPLITUDE = 0.02f;
        const static int DOWNSAMPLE_SCALE = 4;

        static MySemaphore* read_semaphore;
        static MySemaphore* klt_semaphore;
        static MySemaphore* mc_semaphore;
        static MySemaphore* out_semaphore;
        //果冻效应信号量
        static MySemaphore* rs_semaphore;

        static vector<Point2i> kltList;
        static vector<Point2i> motionCompList;
        static vector<Point2i> outputList;
        static list<list<vector<Point2f>>> trj;
        static Mat frameVec[BUFFERSIZE];
        static Mat stableTransformVec[BUFFERSIZE];
        static Mat stableRVec[BUFFERSIZE];

        //果冻效应相关
        const static int rsStripNum = 10;
        static Mat rsOutTheta[BUFFERSIZE];
        static vector<Point2i> rsList;
        static Mat rsMat[BUFFERSIZE][rsStripNum];

        static void Init();
        static void Release();
    };
}


#endif //VIDEOSTABLE_THREADCONTEXT_H
