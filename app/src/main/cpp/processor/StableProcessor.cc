//
// Created by 张哲华 on 19/09/2017.
//

#include "StableProcessor.h"
#include <android/log.h>

using namespace threads;

StableProcessor::~StableProcessor() {
    if (klt_thread_ != nullptr) {
        delete klt_thread_;
        klt_thread_ = nullptr;
    }
    if (cm_thread_ != nullptr) {
        delete cm_thread_;
        cm_thread_ = nullptr;
    }
    if (another_klt_thread_ != nullptr) {
        delete another_klt_thread_;
        another_klt_thread_ = nullptr;
    }
    if(rs_thread_!= nullptr){
        delete rs_thread_;
        rs_thread_ = nullptr;
    }
}

void StableProcessor::Init(Size videoSize) {
    ThreadContext::Init();
    cur_read_count_ = 0;//当前读入计数
    buffer_index_ = 0;//framevec的索引，最大是20
    frame_index_ = 0;//帧索引
    is_first_read_group_ = true;
    klt_thread_ = new ThreadKlt();//光流线程操作
    cm_thread_ = new ThreadCompensation();//运动补偿线程操作
    rs_thread_ = new ThreadRollingShutter();
//    another_klt_thread_ = new ThreadKlt();
    this_rgroup_size_ = ThreadContext::SEGSIZE;

    cur_out_count_ = 0;//输出计数
    klt_thread_ -> videoSize = videoSize;//光流线程的视频大小
//    another_klt_thread_ -> videoSize = videoSize;
    cm_thread_ -> videoSize = videoSize;//运动补偿线程的视频大小
    cm_thread_ -> cropControlFlag = false;//在此设置是否进行裁剪控制
    cm_thread_ -> shakeDetect = false;//在此设置是否进行抖动检测
    cm_thread_ -> cropRation = 0.8;//裁剪率
    //果冻效应
    rs_thread_->videosize = videoSize;


    klt_thread_->start();
    cm_thread_->start();//生成新的cm线程
    rs_thread_->start();
}

void StableProcessor::enqueueInputBuffer(int buffer_index, const Mat* new_frame, const Mat* RR, const Mat* rsOutTheta) {


    if (cur_read_count_ == 0) {  // start of a group
        this_rgroup_start_ = (buffer_index_ - 1 + ThreadContext::BUFFERSIZE) % ThreadContext::BUFFERSIZE;
        this_rgroup_size_ = ThreadContext::SEGSIZE;//该组大小为5
        if (is_first_read_group_) {
            this_rgroup_start_ = buffer_index_;
        }//第一次则开始位置为0
    }

    if (buffer_index != -1) {
        //assert命令：判断括号内表达式是否为真，若为假则发出警告并停止程序
        assert (buffer_index == buffer_index_);
        new_frame->copyTo(ThreadContext::frameVec[buffer_index_]);//放入一帧新图像，frameVec大小为20
        RR->copyTo(ThreadContext::stableRVec[buffer_index_]);//RR是3*3空矩阵，放入stableRVec
        //果冻效应相关
        rsOutTheta->copyTo(ThreadContext::rsOutTheta[buffer_index_]);
        cur_read_count_ ++;//当前读入数量加一
    } else {
        cur_read_count_ ++;
        this_rgroup_size_ = cur_read_count_;
    }
//如果是第一次，并且当前读入数量是该组数量，或者不是第一次读入，并且当前读入数量是该组数量减一
    if ((is_first_read_group_ && cur_read_count_ == this_rgroup_size_)
        || (!is_first_read_group_ && cur_read_count_ == this_rgroup_size_ - 1)) {
        Point2i buffer_info;
        buffer_info.x = this_rgroup_start_;//该组开始的位置
        buffer_info.y = this_rgroup_size_;//该组的大小
        //将位置放入各自标记位置的vector中
        ThreadContext::kltList.push_back(buffer_info);
        ThreadContext::motionCompList.push_back(buffer_info);
        ThreadContext::outputList.push_back(buffer_info);
        //果冻效应相关
        ThreadContext::rsList.push_back(buffer_info);
        //激活klt线程

        ThreadContext::klt_semaphore->Signal();
        //当前读入数量归0
        cur_read_count_ = 0;
        is_first_read_group_ = false;
    }
    //第二组
    buffer_index_ = (buffer_index_ + 1) % ThreadContext::BUFFERSIZE;
//    __android_log_print(ANDROID_LOG_ERROR,
//                        "StableProcessor", "cur_read_count:%d,buffer_index:%d, this_rgroup_start:%d, isfirst:%d, this_size:%d",
//                        cur_read_count_, buffer_index, this_rgroup_start_, is_first_read_group_ ,this_rgroup_size_);
//    __android_log_print(ANDROID_LOG_ERROR, "StableProcessor", "framevec:%d", sizeof(ThreadContext::frameVec)/
//                                                                             sizeof(ThreadContext::frameVec[0]));
}

int StableProcessor::dequeueInputBuffer() {
    //如果当前读入数量为0，就让读入线程停止
    if (cur_read_count_ == 0) {
        ThreadContext::read_semaphore->Wait();
    }
    return buffer_index_;
}


void StableProcessor::enqueueOutputBuffer() {
    cur_out_count_ ++;
    if (cur_out_count_ == this_ogroup_size_ - 1) {
        cur_out_count_ = 0;
        ThreadContext::read_semaphore->Signal();
    }
}

void StableProcessor::dequeueOutputBuffer(Mat* const stableVec, Mat* const frame, Mat* const rsMat) {
    if (cur_out_count_ == 0) {
        ThreadContext::out_semaphore->Wait();

        this_ogroup_start_ = ThreadContext::outputList[0].x;
        this_ogroup_size_ = ThreadContext::outputList[0].y;
        ThreadContext::outputList.erase(ThreadContext::outputList.begin());
    }

    if (ThreadContext::stableTransformVec[0].cols == 0) {
        __android_log_print(ANDROID_LOG_ERROR, "NStableProcessor", "stableTransformVec[0] is empty");
    }
//    frame_index_ ++;
    int index = (this_ogroup_start_ + cur_out_count_) % ThreadContext::BUFFERSIZE;

    ThreadContext::stableTransformVec[index].copyTo(*stableVec);
    //果冻效应
    Mat outTemp(30,3,CV_64F);
    int tempPosition = 0;
    for(int i = 0; i < ThreadContext::rsStripNum; i++){
//        __android_log_print(ANDROID_LOG_ERROR, "StableProcessor:","%drsMat111111:%f", i, ThreadContext::rsMat[index][i].at<double>(0,0));
        Mat temp(3, 3, CV_64F);
        ThreadContext::rsMat[index][i].copyTo(temp);
        for(int j = tempPosition, l = 0; j < tempPosition+3; j++, l++){
            for(int k = 0; k < 3; k++){
                outTemp.at<double>(j, k) = temp.at<double>(l, k);
            }
        }
        tempPosition+=3;
//        __android_log_print(ANDROID_LOG_ERROR, "StableProcessor:","%drsMat111111:%f", i, outTemp.at<double>(tempPosition-3, 0));
    }
    outTemp.copyTo(*rsMat);

    ThreadContext::frameVec[index].copyTo(*frame);
//    putText(*frame, "(0,0)", Point(100,100), cv::FONT_HERSHEY_PLAIN, 3.0, Scalar(0), 10);
//    putText(*frame, "(1920,1080)", Point(1600,900), cv::FONT_HERSHEY_PLAIN, 3.0, Scalar(0), 10);

}