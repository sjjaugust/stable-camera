//
// Created by 张哲华 on 19/09/2017.
//

#ifndef VIDEOSTABLE_STABLEPROCESSOR_H
#define VIDEOSTABLE_STABLEPROCESSOR_H

#include "ThreadKlt.h"
#include "ThreadCompensation.h"
#include "ThreadContext.h"
#include <assert.h>

namespace threads {
    class StableProcessor {
    private:

        ThreadCompensation* cm_thread_;

        volatile int buffer_index_ = 0;
        volatile int out_index_ = 0;
        bool is_first_frame_ = true;

        volatile int frame_index_;

//        volatile int out_buffer_start_ = 0;
    public:
        StableProcessor() :cm_thread_(nullptr){};
        ~StableProcessor();

        void Init(Size videoSize);
        int dequeueInputBuffer();
        void enqueueInputBuffer(int buffer_index, const Mat* new_frame, const Mat* RR);
        void enqueueOutputBuffer();
        void dequeueOutputBuffer(Mat* const stableVec, Mat* const frame);
        void setCrop(bool isCrop);
    };
}


#endif //VIDEOSTABLE_STABLEPROCESSOR_H
