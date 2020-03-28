//
// Created by ShiJJ on 2020/3/27.
//

#include "TimeRecorder.h"
TimeRecorder* TimeRecorder::ms_TimeRecorder = nullptr;
TimeRecorder& TimeRecorder::GetTimeRecord() {
    if(ms_TimeRecorder == nullptr){
        ms_TimeRecorder = new TimeRecorder;
    }
    return *ms_TimeRecorder;
}
void TimeRecorder::SetTimeStart() {
    m_TimeStart = clock();
}
void TimeRecorder::SetTimeEnd() {
    m_TimeEnd = clock();
    m_TimeSpend.push_back((double)(m_TimeEnd - m_TimeStart)/CLOCKS_PER_SEC);
    __android_log_print(ANDROID_LOG_DEBUG, "TimeRecorder", "[DEBUG]:%d, %f", m_TimeSpend.size(), m_TimeSpend[m_TimeSpend.size()-1]);
}
void TimeRecorder::LogAverageTime() {
    float timeTemp = 0.0f;
    for(auto it : m_TimeSpend){
        timeTemp += it;
    }
    timeTemp /= m_TimeSpend.size();
    __android_log_print(ANDROID_LOG_DEBUG, "TimeRecorder", "[DEBUG]:Average Time is: %f", timeTemp);
}
bool TimeRecorder::IsTimeSpendOver(int timeSpendSize) {
    return m_TimeSpend.size() == timeSpendSize;
}
TimeRecorder::~TimeRecorder() {
//    if(TimeRecorder::ms_TimeRecorder != nullptr){
//        delete TimeRecorder::ms_TimeRecorder;
//    }

}