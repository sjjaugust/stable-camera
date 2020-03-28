//
// Created by 张哲华 on 19/09/2017.
//

#include "ThreadKlt.h"
#include <android/log.h>

using namespace threads;
ThreadKlt::~ThreadKlt() {
    worker_thread_.join();
}

void ThreadKlt::start() {
    worker_thread_ = thread(&ThreadKlt::worker, this);
}

void ThreadKlt::worker() {
    pthread_setname_np(pthread_self(), TAG); // set the name (pthread_self() returns the pthread_t of the current thread)
    while(true)
    {
        //count为0，让该线程等待，signal后，count为1，继续该线程

        ThreadContext::klt_semaphore->Wait();//取原始帧容器中图像资源，若无，则线程等待

//        __android_log_print(ANDROID_LOG_DEBUG, "NThreadKlt", "before");

        if( ThreadContext::kltList[0].y < ThreadContext::SEGSIZE )
        {
            ThreadContext::mc_semaphore->Signal();
            break;
        }
        constructTrajectory();

        ThreadContext::kltList.erase( ThreadContext::kltList.begin() );
//        __android_log_print(ANDROID_LOG_DEBUG, "ThreadKlt", "timespand:%f, %d",(float)(c_finish-c_start)/CLOCKS_PER_SEC, timeSpand.size());

        ThreadContext::mc_semaphore->Signal();//唤醒运动估计线程
    }
}

//判断特征点坐标是否超出图像范围
bool ThreadKlt::outOfImg(const Point2f &point, const Size &size)
{
    return (point.x <= 0 || point.y <= 0 || point.x >= size.width - 1 || point.y >= size.height - 1 );
}

//判断两个特征点是否为同一点，当其坐标十分接近时认为是同一点
bool ThreadKlt::isTheSame(const Point2f &point, const Point2f &pt)
{
    return (point.x - pt.x)*(point.x - pt.x)+(point.y - pt.y)*(point.y - pt.y) < ThreadContext::MIN_DISTANCE*ThreadContext::MIN_DISTANCE/10;
}

//使用opencv封装好的KLT算法进行运动估计，即构造特征点轨迹，从start处开始
void ThreadKlt::constructTrajectory()
{
    int start = ThreadContext::kltList[0].x;//视频段起点在容器中的位置
    int length = ThreadContext::kltList[0].y;//视频段长度

    list<vector<Point2f>> features;//存储特征点轨迹
    Mat preGray , curGray;
    vector<Point2f> preFeats;
    vector<Point2f> curFeats;
    vector<uchar> status;
    vector<float> err;
    preGray = ThreadContext::frameVec[start](Range(0, videoSize.height), Range(0, videoSize.width));
//    cvtColor( ThreadContext::frameVec[start] , preGray , CV_RGB2GRAY );//转化为灰度图
    Size newSize = videoSize;
    if( videoSize == Size(1920,1080) )
    {
        //下采样尺度为4
        newSize = Size(preGray.cols / ThreadContext::DOWNSAMPLE_SCALE,
                       preGray.rows / ThreadContext::DOWNSAMPLE_SCALE);
        resize(preGray, preGray, newSize);//高分辨率视频降采样

    }
    //输入图像为降采样后的图像，第二个参数为存放特征点的数组，第三个参量为特征点最大个数，第四个参数为质量水平，两个特征点的最小距离20
    goodFeaturesToTrack( preGray , preFeats , 60 , 0.1 , ThreadContext::MIN_DISTANCE );//检测特征点
    //特征点个数
    int preFeatsNum = preFeats.size();
    //遍历第一幅图特征点，乘以RR,变成卡尔曼滤波后的坐标点，存入feature
    for( int i = 0 ; i < preFeatsNum ; i++ )
    {
        vector<Point2f> tmp;

        //特征点乘了RR
        Mat p3=Mat(3,1,CV_64F);
        p3.at<float>(0,0)=preFeats[i].x;
        p3.at<float>(1,0)=preFeats[i].y;
        p3.at<float>(2,0)=1.0;
        Mat p3new=ThreadContext::stableRVec[start]*p3;
        preFeats[i].x=p3new.at<float>(0,0)/p3new.at<float>(2,0);
        preFeats[i].y=p3new.at<float>(1,0)/p3new.at<float>(2,0);


        tmp.push_back( preFeats[i] );//放入一个点
        features.push_back( tmp );
    }

    for( int i = 1 ; i < length ; i++ )
    {
        int index = ( start + i ) % ( ThreadContext::BUFFERSIZE );


        curFeats.clear();
        status.clear();
        err.clear();
        curGray = ThreadContext::frameVec[index](Range(0, videoSize.height), Range(0, videoSize.width));
//        cvtColor( ThreadContext::frameVec[index] , curGray , CV_RGB2GRAY );

        if( videoSize == Size(1920,1080) )//高分辨率视频降采样
        {
            resize(curGray, curGray, newSize);
        }

        if (preFeats.size() == 0)
            break;

        //不需要后一帧的特征点，计算出下一帧的匹配点
        calcOpticalFlowPyrLK( preGray , curGray , preFeats , curFeats , status , err );//根据已检测到的前一帧特征点在后一帧查找匹配的特征点

        /*消除特征点误匹配*/
        vector<Point2f> prefeatures = preFeats;
        vector<Point2f> curfeatures = curFeats;
        int ptCount = status.size();
        Mat p1(ptCount, 2, CV_32F);
        Mat p2(ptCount, 2, CV_32F);
        for (int j = 0 ; j < ptCount ; j++ )
        {
            p1.at<float>(j,0) = prefeatures[j].x;//前一帧特征点
            p1.at<float>(j,1) = prefeatures[j].y;

            p2.at<float>(j,0) = curfeatures[j].x;//后一帧特征点
            p2.at<float>(j,1) = curfeatures[j].y;
//            __android_log_print(ANDROID_LOGINFO, "NThreadKlt", "features x%f y%f", prefeatures[j].x, prefeatures[j].y);

        }
        Mat m_Fundamental;
        vector<uchar> m_RANSACStatus;
        m_Fundamental = findFundamentalMat(p1, p2, m_RANSACStatus, FM_RANSAC);
        for (int j = 0 ; j < ptCount ; j++ )
        {
            if (m_RANSACStatus.size() > j && m_RANSACStatus[j] == 0) // 状态为0表示野点(误匹配)
            {
                status[j] = 0;
            }
        }



        list<vector<Point2f>>::iterator Iter;
        int j = 0;
        preFeats.clear();
        for( Iter = features.begin() ; Iter != features.end() ; j++ )
        {
            if(status[j] == 0 || err[j] > 20 || outOfImg(curFeats[j], newSize))
            {
                Iter = features.erase( Iter );//消除存储一个点的vector
            }
            else
            {
                //特征点乘了RR！
                Mat p3=Mat(3,1,CV_64F);
                p3.at<float>(0,0)=curFeats[i].x;
                p3.at<float>(1,0)=curFeats[i].y;
                p3.at<float>(2,0)=1.0;
                Mat p3new=ThreadContext::stableRVec[index]*p3;//当前帧乘转换矩阵
                curFeats[i].x=p3new.at<float>(0,0)/p3new.at<float>(2,0);
                curFeats[i].y=p3new.at<float>(1,0)/p3new.at<float>(2,0);

                preFeats.push_back( curFeats[j]);
                (*Iter).push_back( curFeats[j]);//vector增加现在的点
                Iter++;
            }
        }

        if( features.size() == 0 )
        {
            break;
        }

        preGray = curGray.clone();
    }

    ThreadContext::trj.push_back( features );

}