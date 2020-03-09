//
// Created by 张哲华 on 20/09/2017.
//

#ifndef VIDEOSTABLE_SEMAPHORE_H
#define VIDEOSTABLE_SEMAPHORE_H

#include <mutex>
#include <condition_variable>
namespace threads {
    class MySemaphore {
    public:
        MySemaphore(int count = 1) : count_{count} {}//把count给私有变量count_
        void Wait();
        void Signal();
        bool TryWait();

    private:
        int count_;
        std::mutex mutex_;
        std::condition_variable condition_;
    };
}

#endif //VIDEOSTABLE_SEMAPHORE_H
