//
// Created by 张哲华 on 19/09/2017.
//

#ifndef VIDEOSTABLE_STABLEPROCESSOR_H
#define VIDEOSTABLE_STABLEPROCESSOR_H

#include "ThreadKlt.h"
#include "ThreadCompensation.h"
#include "ThreadContext.h"
#include "ThreadRollingShutter.h"
#include <assert.h>

namespace threads {
    class StableProcessor {
    private:
        ThreadKlt* klt_thread_;
        ThreadKlt* another_klt_thread_;
        ThreadCompensation* cm_thread_;
        //果冻效应
        ThreadRollingShutter* rs_thread_;
        volatile int cur_read_count_;
        bool is_first_read_group_ = true;
        volatile int buffer_index_ = 0;
        volatile int this_rgroup_size_;
        volatile int this_rgroup_start_;

        volatile int frame_index_;

//        volatile int out_buffer_start_ = 0;
        volatile int cur_out_count_;
        volatile int this_ogroup_size_;
        volatile int this_ogroup_start_;
    public:
        StableProcessor() : klt_thread_(nullptr),
                            cm_thread_(nullptr),
                            another_klt_thread_(nullptr),
                            rs_thread_(nullptr){};
        ~StableProcessor();

        void Init(Size videoSize);
        int dequeueInputBuffer();
        void enqueueInputBuffer(int buffer_index, const Mat* new_frame, const Mat* RR, const Mat* rsOutTheta);
        void enqueueOutputBuffer();
        void dequeueOutputBuffer(Mat* const stableVec, Mat* const frame, Mat* rsMat);
    };
}


#endif //VIDEOSTABLE_STABLEPROCESSOR_H
