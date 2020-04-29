//
// Created by ShiJJ on 2020/4/16.
//

#include "StableProcessor.h"

void StableProcessor::Init() {
    ThreadContext::Init();
    cm_thread_ = new ThreadCompensation();
    cm_thread_->Start();
}
StableProcessor::~StableProcessor() {
    //TODO destructor
    ThreadContext::Release();
    if(cm_thread_ != nullptr){
        delete cm_thread_;
    }


}
int StableProcessor::DequeueInputBuffer() {
    ThreadContext::read_semaphore_->Wait();
    return in_index_;
}
void StableProcessor::EnqueueOutputBuffer() {
    ThreadContext::read_semaphore_->Singal();
}
void StableProcessor::EnqueueInputBuffer(int buffer_index, const cv::Mat *new_frame,
                                         const cv::Mat* quaternion_mat) {
    assert(buffer_index==in_index_);
    new_frame->copyTo(ThreadContext::frame_vec_[in_index_]);
    quaternion_mat->copyTo(ThreadContext::quaternion_vec_[in_index_]);
    if(is_first){
        is_first = false;
    } else {
        ThreadContext::cm_semaphore_->Singal();
    }
    in_index_ = (in_index_ + 1) % ThreadContext::BUFFER_SIZE_;

}
void StableProcessor::DequeueOutputBuffer(cv::Mat *const stableVec, cv::Mat *const frame, cv::Mat* const rs_convert_mat) {
    ThreadContext::out_semaphore_->Wait();
    ThreadContext::stable_vec_[out_index_].copyTo(*stableVec);
    ThreadContext::frame_vec_[out_index_].copyTo(*frame);
    ThreadContext::rs_convert_mat_.copyTo(*rs_convert_mat);
    out_index_ = (out_index_ + 1) % ThreadContext::BUFFER_SIZE_;
}
void StableProcessor::SetCrop(bool is_crop) {
    cm_thread_->crop_control_flag = is_crop;
}