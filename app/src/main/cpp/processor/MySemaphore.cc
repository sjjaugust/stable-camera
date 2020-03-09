//
// Created by 张哲华 on 20/09/2017.
//

#include "MySemaphore.h"
using namespace threads;

//如果count为0，就让锁控制的线程等待，直到被唤醒，count为剩余线程数
void MySemaphore::Wait() {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    while(!count_) // Handle spurious wake-ups.
        condition_.wait(lock);
    --count_;
}
//唤醒该锁控制的线程
void MySemaphore::Signal() {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    ++count_;
    condition_.notify_one();
}
bool MySemaphore::TryWait() {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    if(count_) {
        --count_;
        return true;
    }
    return false;
}
