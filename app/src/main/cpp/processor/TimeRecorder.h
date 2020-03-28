//
// Created by ShiJJ on 2020/3/27.
//

#ifndef STABLE_CAMERA_TIMERECORDER_H
#define STABLE_CAMERA_TIMERECORDER_H

#include <ctime>
#include <vector>
#include <android/log.h>
using namespace std;
class TimeRecorder{
private:
    static TimeRecorder* ms_TimeRecorder;
    clock_t m_TimeStart, m_TimeEnd;
    vector<double> m_TimeSpend;
public:
    static TimeRecorder& GetTimeRecord();
    void SetTimeStart();
    void SetTimeEnd();
    void LogAverageTime();
    bool IsTimeSpendOver(int timeSpendSize);
    ~TimeRecorder();

};

#endif //STABLE_CAMERA_TIMERECORDER_H
