//
// Created by ShiJJ on 2020/4/16.
//

#ifndef STABLE_CAMERA_MYSEMAPHORE_H
#define STABLE_CAMERA_MYSEMAPHORE_H

#include <thread>
#include <mutex>
#include <condition_variable>

class MySemaphore {
private:
    int count_;
    std::mutex mutex_;
    std::condition_variable condition_;
public:
    MySemaphore(const int count = 0);
    void Wait();
    void Singal();
    void SingalAll();
};


#endif //STABLE_CAMERA_MYSEMAPHORE_H
