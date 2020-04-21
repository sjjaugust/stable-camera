//
// Created by ShiJJ on 2020/4/16.
//

#include "MySemaphore.h"
MySemaphore::MySemaphore(const int count)
: count_(count){}
void MySemaphore::Wait() {
    std::unique_lock<std::mutex> lock(mutex_);
//    if(--count_ < 0){
//        condition_.wait(lock);
//    }
    while(!count_) // Handle spurious wake-ups.
        condition_.wait(lock);
    --count_;
}
void MySemaphore::Singal() {
    std::unique_lock<std::mutex> lock(mutex_);
//    if(++count_ <= 0){
//        condition_.notify_one();
//    }
    count_++;
    condition_.notify_one();
}
void MySemaphore::SingalAll() {
    std::unique_lock<std::mutex> lock;
    if(count_ < 0){
        condition_.notify_all();
        count_ = 0;
    }
}