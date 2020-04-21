//
// Created by ShiJJ on 2020/4/18.
//

#include "ThreadCompensation.h"
ThreadCompensation::~ThreadCompensation() {
    work_thread_.join();
}
void ThreadCompensation::Start() {
    work_thread_ = std::thread(&ThreadCompensation::Work, this);
}
void ThreadCompensation::Work() {
    pthread_setname_np(pthread_self(), "CompensationThread");
    filiter = Filiter(10, 10);
    while (true){
        ThreadContext::cm_semaphore_->Wait();
        if(cm_index_ < 0){
            ThreadContext::out_semaphore_->Singal();
            break;
        }
        FrameCompensation();
    }
}
void ThreadCompensation::FrameCompensation() {
    Quaternion first_q(ThreadContext::quaternion_vec_[cm_index_].at<double>(0, 1),
                       ThreadContext::quaternion_vec_[cm_index_].at<double>(0, 2),
                       ThreadContext::quaternion_vec_[cm_index_].at<double>(0, 2),
                       ThreadContext::quaternion_vec_[cm_index_].at<double>(0, 3));
    bool ready_to_pull = filiter.Push(first_q);
    q_cache_.push(first_q);
    if(ready_to_pull){
        Quaternion new_q = filiter.Pop();
        Quaternion old_q = q_cache_.front();
        q_cache_.pop();
        Quaternion convert_q = Quaternion::Q1ToQ2(old_q, new_q);
        cv::Mat convert_mat = Quaternion::QuaternionToR(convert_q);
        convert_mat = ThreadContext::inmat * convert_mat * ThreadContext::inmat.inv();
        convert_mat = ThreadContext::RR2stableVec * convert_mat * ThreadContext::stableVec2RR;
        convert_mat.copyTo(ThreadContext::stable_vec_[out_index_]);
        ThreadContext::out_semaphore_->Singal();
        out_index_ = (out_index_ + 1) % ThreadContext::BUFFER_SIZE_;
    }
    cm_index_ = (cm_index_ + 1) % ThreadContext::BUFFER_SIZE_;


}
