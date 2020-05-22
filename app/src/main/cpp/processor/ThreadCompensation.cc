//
// Created by 张哲华 on 19/09/2017.
//

#include "ThreadCompensation.h"

#include <android/log.h>
#include <jni.h>
#include <opencv2/core/mat.hpp>

#define LOG_TAG    "c_ThreadCompensation"
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

using namespace cv;
using namespace std;
using namespace threads;
static int frame_count = 0;
static cv::Mat point_before = (cv::Mat_<double>(3, 1) << 960, 540, 0);
static cv::Mat point_after = (cv::Mat_<double>(3, 1) << 960, 540, 0);
static FILE* file_before;
static FILE* file_after;
static std::queue<cv::Mat> trans_que;
static std::queue<cv::Mat> r_temp_queue;
static cv::Mat tran_cumm = cv::Mat::eye(3, 3, CV_64F);
double point_distance(cv::Point2f p1,cv::Point2f p2)
{
    cv::Point2f d = p1 - p2;
    double d_mu;
    d_mu = sqrt(d.x * d.x + d.y * d.y);
    return d_mu;
}

bool outOfImg(const cv::Point2f &point, const cv::Size &size)
{
    return (point.x <= 0 || point.y <= 0 || point.x >= size.width - 1 || point.y >= size.height - 1 );
}

ThreadCompensation::~ThreadCompensation() {
    worker_thread_.join();
}

void ThreadCompensation::start() {
    file_before = fopen("/data/data/me.zhehua.gryostable/data/track_before.txt", "a");
    file_after = fopen("/data/data/me.zhehua.gryostable/data/track_after.txt", "a");
    filter = Filter(10, 20, Filter::delta_T);
    last_homography_ = cv::Mat::eye(3, 3, CV_64F);
    worker_thread_ = thread(&ThreadCompensation::worker, this);
}

void ThreadCompensation::worker()
{
    //LOGI("ThreadCompensation::worker");
//    filter = Filter(ThreadContext::SEGSIZE * 2 , 5);
    lastRot[0]=0;
    lastRot[1]=0;
    lastRot[2]=0;
    //cropRation = 0.8;
    pthread_setname_np(pthread_self(), "CompensationThread"); // set the name (pthread_self() returns the pthread_t of the current thread
    while(true)
    {
        //LOGI("loop start");
        ThreadContext::mc_semaphore->Wait();//取已经完成特征点轨迹构造的资源，若无，线程等待
//        __android_log_print(ANDROID_LOG_DEBUG, "NThreadMC", "before");
        if( cm_las_index_ < 0 )
        {
            ThreadContext::out_semaphore->Signal();
            break;
        }

        frameCompensate();
        //LOGI("loop end");

        ex_index_ = (ex_index_ + 1) % ThreadContext::SEGSIZE;
        cm_las_index_ = (cm_las_index_ + 1) % ThreadContext::BUFFERSIZE;
        cm_cur_index_ = (cm_cur_index_ + 1) % ThreadContext::BUFFERSIZE;
        //ThreadContext::out_semaphore->Signal();//唤醒显示和保存线程

    }
}

void ThreadCompensation::detect_feature()
{
    double quality_level = 0.1;
    double min_distance = 8;
    int max_corners = 8;

    std::vector<cv::Point2f> lastFeatures_a[16], startp;
    lastFeatures.clear();

    int half_w=lastGray.cols/4 , half_h=lastGray.rows/4;
    startp.push_back(cv::Point2f(0, 0));
    startp.push_back(cv::Point2f(0, half_h));
    startp.push_back(cv::Point2f(0, 2*half_h));
    startp.push_back(cv::Point2f(0, 3*half_h));

    startp.push_back(cv::Point2f(half_w, 0));
    startp.push_back(cv::Point2f(half_w, half_h));
    startp.push_back(cv::Point2f(half_w, 2*half_h));
    startp.push_back(cv::Point2f(half_w, 3*half_h));

    startp.push_back(cv::Point2f(2*half_w, 0));
    startp.push_back(cv::Point2f(2*half_w, half_h));
    startp.push_back(cv::Point2f(2*half_w, 2*half_h));
    startp.push_back(cv::Point2f(2*half_w, 3*half_h));

    startp.push_back(cv::Point2f(3*half_w, 0));
    startp.push_back(cv::Point2f(3*half_w, half_h));
    startp.push_back(cv::Point2f(3*half_w, 2*half_h));
    startp.push_back(cv::Point2f(3*half_w, 3*half_h));

    cv::Rect rect[16];
    cv::Mat lastGray_a[16];

    for(int i=0;i<16;i++)
    {
        rect[i]=cv::Rect(startp[i].x, startp[i].y, half_w, half_h);//左上角坐标以及矩形宽和高
        lastGray_a[i] = lastGray(rect[i]);
        goodFeaturesToTrack(lastGray_a[i], lastFeatures_a[i], max_corners, quality_level, min_distance);//检测特征点
        for(int j=0; j<lastFeatures_a[i].size(); j++)
        {
            cv::Point2f pt=lastFeatures_a[i][j]+startp[i];//转化成全局坐标
            lastFeatures.push_back(pt);
        }
    }
}

void ThreadCompensation::track_feature()
{
    double rate = 1.4;

    status.clear();
    std::vector<float> err;
    curFeatures.clear();

    //LOGI("step4_1");
    calcOpticalFlowPyrLK( lastGray , curGray , lastFeatures , curFeatures , status , err);//根据已检测到的前一帧特征点在后一帧查找匹配的特征点
    //如果没找到匹配点会将前一帧特征点位置复制到curFeatures中，并在status中标记为0
    status_choose.clear();
    status_choose.assign(status.begin(), status.end());//将status复制到status_choose中

    //LOGI("step4_2");
    int max = lastFeatures.size() < curFeatures.size() ? lastFeatures.size() : curFeatures.size();
    double dis_sum=0;
    //如果是没有找到匹配点，两点之间的距离为0，不影响平均距离
    for(int i=0;i<max;i++)
    {
        dis_sum += point_distance(lastFeatures[i],curFeatures[i]);
    }
    double dis_avg=0;
    dis_avg=dis_sum/max;

    //LOGI("step4_3");
    max = max < status.size() ? max : status.size();
    for(int i=0;i<max;i++)
    {
        if(point_distance(lastFeatures[i],curFeatures[i]) > dis_avg * rate)
        {
            status[i] = 0;
        }
    }//如果大于特征点之间的距离大于平均距离的1.4倍，则舍弃

    //LOGI("step4_4");
    cv::Mat m_Fundamental;
    std::vector<uchar> m_RANSACStatus;
    cv::Mat p1(lastFeatures);
    cv::Mat p2(curFeatures);
    double outliner=0;
    m_Fundamental = findFundamentalMat(p1, p2, m_RANSACStatus, cv::FM_RANSAC, 3, 0.99);

    //LOGI("step4_5");
    for (int j = 0 ; j < status.size() ; j++ )
    {
        if (m_RANSACStatus.size() > j && m_RANSACStatus[j] == 0) // 状态为0表示野点(误匹配)
        {
            status[j] = 0;
        }
        if(status[j]==0)
        {
            outliner++;
        }
    }

    int idx = 0;
    curFeaturesTmp.clear();
    lastFeaturesTmp.clear();

    //LOGI("step4_6");
    for (auto itC = curFeatures.begin(), itP = lastFeatures.begin(); itC != curFeatures.end(); itC ++, itP ++, idx ++) {

        if (status[idx] == 0 || err[idx] > 20 || outOfImg(*itC, Size(lastGray.cols, lastGray.rows))) {
            status_choose[idx]=0;
        } else {
            cv::Point2f cfp=*itC * ThreadContext::DOWNSAMPLE_SCALE;
            cv::Point2f lfp=*itP * ThreadContext::DOWNSAMPLE_SCALE;
            curFeaturesTmp.push_back(cfp);
            lastFeaturesTmp.push_back(lfp);
        }
    }
    feature_by_r_.push(curFeaturesTmp);
}

Mat ThreadCompensation::moveAndScale()
{
    double hw = lastGray.cols * ThreadContext::DOWNSAMPLE_SCALE / 2;
    double hh = lastGray.rows * ThreadContext::DOWNSAMPLE_SCALE / 2;
    int poi_count[3];
    Point2f last_poi_avg[3], cur_poi_avg[3];

    for(int i=0; i<3; i++)
    {
        last_poi_avg[i] = Point2f(0,0);
        cur_poi_avg[i] = Point2f(0,0);
        poi_count[i] = 0;
    }

    int n = lastFeaturesTmp.size() < curFeaturesTmp.size() ? lastFeaturesTmp.size() : curFeaturesTmp.size();
    for(int i=0; i<n; i++)
    {
        if(lastFeaturesTmp[i].y < hh)
        {
            poi_count[0]++;
            last_poi_avg[0] += lastFeaturesTmp[i];
            cur_poi_avg[0] += curFeaturesTmp[i];
        }
        else
        {
            poi_count[1]++;
            last_poi_avg[1] += lastFeaturesTmp[i];
            cur_poi_avg[1] += curFeaturesTmp[i];
        }

        poi_count[2]++;
        last_poi_avg[2] += lastFeaturesTmp[i];
        cur_poi_avg[2] += curFeaturesTmp[i];
    }

    bool all_ok = true;
    for(int i=0; i<3; i++)
    {
        if(poi_count[i] <= 0)
        {
            all_ok = false;
            break;
        }
        last_poi_avg[i] = last_poi_avg[i] / poi_count[i];
        cur_poi_avg[i] = cur_poi_avg[i] / poi_count[i];
    }

    Mat move = Mat::eye(3,3,CV_64F);
    if(!all_ok)
    {
        return move;
    }
    Mat scale = Mat::eye(3,3,CV_64F);
    Point2f m = cur_poi_avg[2] - last_poi_avg[2];
    double l[3];
    l[0] = point_distance(last_poi_avg[0], last_poi_avg[1]);//前一帧上半部分和下半部分特征点平均值之间的距离
    l[1] = point_distance(cur_poi_avg[0], cur_poi_avg[1]);//后一帧上半部分和下半部分特征点平均值之间的距离
    l[2] = l[1]/l[0];//距离之比
    scale.at<double>(0,0) = l[2];
    scale.at<double>(1,1) = l[2];
    move.at<double>(0,2) = m.x;
    move.at<double>(1,2) = m.y;

//    test_point_before = scale * move *test_point_before;
//    cv::Point2d temp_point;
//    temp_point.x = test_point_before.at<double>(0, 0);
//    temp_point.y = test_point_before.at<double>(1, 0);
//    __android_log_print(ANDROID_LOG_DEBUG, "ThreadCompensation", "before:%d, %f", frame_count, point_distance(temp_point, test_point1));
//    __android_log_print(ANDROID_LOG_DEBUG, "ThreadCompensation", "before:%d, %f", frame_count, temp.at<double>(0, 0));

    return (scale * move);
}

bool ThreadCompensation::affPointSimplify( vector<Point2f> &last_out , vector<Point2f> &cur_out )
{
    double hw = lastGray.cols * ThreadContext::DOWNSAMPLE_SCALE / 2;
    double hh = lastGray.rows * ThreadContext::DOWNSAMPLE_SCALE / 2;
    int poi_count[3];
    Point2f last_poi_avg[3], cur_poi_avg[3];
    last_out.clear();
    cur_out.clear();

    for(int i=0; i<3; i++)
    {
        last_poi_avg[i] = Point2f(0,0);
        cur_poi_avg[i] = Point2f(0,0);
        poi_count[i] = 0;
    }

    int n = lastFeaturesTmp.size() < curFeaturesTmp.size() ? lastFeaturesTmp.size() : curFeaturesTmp.size();
    for(int i=0; i<n; i++)
    {
        if(lastFeaturesTmp[i].y < hh)
        {
            poi_count[0]++;
            last_poi_avg[0] += lastFeaturesTmp[i];
            cur_poi_avg[0] += curFeaturesTmp[i];
        }
        else if(lastFeaturesTmp[i].x < hw)
        {
            poi_count[1]++;
            last_poi_avg[1] += lastFeaturesTmp[i];
            cur_poi_avg[1] += curFeaturesTmp[i];
        }
        else
        {
            poi_count[2]++;
            last_poi_avg[2] += lastFeaturesTmp[i];
            cur_poi_avg[2] += curFeaturesTmp[i];
        }
    }

    bool all_ok = true;
    for(int i=0; i<3; i++)
    {
        if(poi_count[i] <= 0)
        {
            all_ok = false;
            break;
        }
        last_poi_avg[i] = last_poi_avg[i] / poi_count[i];
        cur_poi_avg[i] = cur_poi_avg[i] / poi_count[i];

        last_out.push_back(last_poi_avg[i]);
        cur_out.push_back(cur_poi_avg[i]);
    }

    return all_ok;
}

bool ThreadCompensation::affPointSimplify_tri( vector<Point2f> &last_out , vector<Point2f> &cur_out )
{
    double hw = lastGray.cols * ThreadContext::DOWNSAMPLE_SCALE / 2;
    double hh = lastGray.rows * ThreadContext::DOWNSAMPLE_SCALE / 2;
    int poi_count[3];
    Point2f last_poi_avg[3], cur_poi_avg[3];
    last_out.clear();
    cur_out.clear();

    for(int i=0; i<3; i++)
    {
        last_poi_avg[i] = Point2f(0,0);
        cur_poi_avg[i] = Point2f(0,0);
        poi_count[i] = 0;
    }

    int n = lastFeaturesTmp.size() < curFeaturesTmp.size() ? lastFeaturesTmp.size() : curFeaturesTmp.size();
    for(int i=0; i<n; i++)
    {
        if(lastFeaturesTmp[i].y < hh)
        {
            poi_count[0]++;
            last_poi_avg[0] += lastFeaturesTmp[i];
            cur_poi_avg[0] += curFeaturesTmp[i];
        }
        else
        {
            poi_count[1]++;
            last_poi_avg[1] += lastFeaturesTmp[i];
            cur_poi_avg[1] += curFeaturesTmp[i];
        }
    }

    bool all_ok = true;
    for(int i=0; i<2; i++)
    {
        if(poi_count[i] <= 0)
        {
            all_ok = false;
            break;
        }
        last_poi_avg[i] = last_poi_avg[i] / poi_count[i];
        cur_poi_avg[i] = cur_poi_avg[i] / poi_count[i];

        last_out.push_back(last_poi_avg[i]);
        cur_out.push_back(cur_poi_avg[i]);
    }

    if(all_ok) {
        Point2f lp = (last_poi_avg[0] + last_poi_avg[1]) / 2;
        Point2f hlp = lp - last_poi_avg[0];
        Point2f trilp;
        trilp.x = lp.x + hlp.y;
        trilp.y = lp.y + hlp.x;

        Point2f cp = (cur_poi_avg[0] + cur_poi_avg[1]) / 2;
        Point2f hcp = cp - cur_poi_avg[0];
        Point2f tricp;
        tricp.x = cp.x + hcp.y;
        tricp.y = cp.y + hcp.x;

        last_out.push_back(trilp);
        cur_out.push_back(tricp);
    }

    return all_ok;
}

Mat ThreadCompensation::calcul_Homo(int niter)
{
    Mat H = cv::Mat();
    std::vector<char> ifselect;
    //std::cout<<totalIdx<<" feature size:"<<lastFeaturesTmp.size()<<std::endl;
    //(*fnp)<<totalIdx<<" "<<lastFeaturesTmp.size()<<endl;
    if(lastFeaturesTmp.size() < 3)
    {
        H = cv::Mat::eye(3, 3, CV_64F);
    }
    else
    {
        vector<Point2f> p1,p2;
        if(with_roll)
        {
            bool ok = affPointSimplify_tri(p1,p2);
            if(ok)
            {
                H = getAffineTransform(p1 , p2);
            } else{
                H = cv::Mat::eye(3, 3, CV_64F);
            }
        } else{
            H = moveAndScale();
        }
    }

    Mat aff = Mat::zeros(3,3,CV_64FC1);
    aff.at<double>(0,0) = H.at<double>(0,0);
    aff.at<double>(0,1) = H.at<double>(0,1);
    aff.at<double>(0,2) = H.at<double>(0,2);
    aff.at<double>(1,0) = H.at<double>(1,0);
    aff.at<double>(1,1) = H.at<double>(1,1);
    aff.at<double>(1,2) = H.at<double>(1,2);
    aff.at<double>(2,2) = 1;

    return aff;
}

double line_distance(cv::Point2f p0, cv::Point2f p1, cv::Point2f p2)
{
    double d = (fabs((p2.y - p1.y) * p0.x +(p1.x - p2.x) * p0.y + ((p2.x * p1.y) -(p1.x * p2.y)))) / (sqrt(pow(p2.y - p1.y, 2) + pow(p1.x - p2.x, 2)));
    return d;
}//中心点到p1p2组成直线的距离

void ThreadCompensation::calcul_Homo_s(std::vector<cv::Point2f> last, std::vector<cv::Point2f> cur) {
    cv::Mat h;
    std::vector<char> s;
    //cout<<totalIdx<<" feature size:"<<lastFeaturesTmp.size()<<endl;

    h = findHomography(last, cur, 0);

    //hom_s.push_back(h);

}
//检测是否抖动
bool ThreadCompensation::stable_count(double e)
{
    double height = curGray.rows * ThreadContext::DOWNSAMPLE_SCALE;
    double width = curGray.cols * ThreadContext::DOWNSAMPLE_SCALE;
    cv::Point2f cen(width/2, height/2);
    double long_side = width > height ? width : height;
    double limit_cor = sqrt(pow(height, 2) + pow(width, 2))/15;

    double sta_sca_limit = 0.00015;
    double sta_limit = long_side * sta_sca_limit;
    double sca_limit = long_side * sta_sca_limit * 4;
    int numStable = 0,numScale = 0;

    std::vector<cv::Point2f> last_sc,cur_sc;

    int num=(curFeatures.size()<lastFeatures.size()?curFeatures.size():lastFeatures.size());
    for(int i=0;i<num;i++)
    {
        cv::Point2f d = curFeatures[i] - lastFeatures[i];
        float d_mu;
        d_mu = sqrt(d.x * d.x + d.y * d.y);
        if (d_mu > sca_limit || status_choose[i]==0)
        {

        }
        else if (d_mu > sta_limit && d_mu <= sca_limit)
        {
            cv::Point2f cp1 = curFeatures[i] * ThreadContext::DOWNSAMPLE_SCALE, lp1 = lastFeatures[i] * ThreadContext::DOWNSAMPLE_SCALE;
            double d=line_distance(cen,cp1,lp1);
            //cout<<"limit:"<<limit_cor<<endl;
            //cout<<"zoom_center_distance: "<<d<<endl;

            if(d<limit_cor)
            {
//                cv::Point2f cp2 = cp1, lp2 = lp1;
//                cp2.x = width - cp1.x;
//                lp2.x = width - lp1.x;
//                cv::Point2f cp3 = cp1, lp3 = lp1;
//                cp3.y = height - cp1.y;
//                lp3.y = height - lp1.y;
//                cv::Point2f cp4 = cp2, lp4 = lp2;
//                cp4.y = height - cp2.y;
//                lp4.y = height - lp2.y;

                cur_sc.push_back(cp1);
                last_sc.push_back(lp1);
//                cur_sc.push_back(cp2);
//                last_sc.push_back(lp2);
//                cur_sc.push_back(cp3);
//                last_sc.push_back(lp3);
//                cur_sc.push_back(cp4);
//                last_sc.push_back(lp4);
                numScale++;
            } else
            {
                //numStable++;
            }
        }
        else
        {
            numStable++;
        }
    }

    LOGE("stable and scale: %d / %d", numStable , numScale);
    if(numStable > 10 || e < 1e-5)
    {
        LOGE("is stable");
        H_scale = cv::Mat::eye(3, 3, CV_64F);
        return true;
    }
    /*else if(numScale > numStable && numScale > 4)
    {
        int n = last_sc.size();
        double nscale = 0;
        for(int i = 0;i<n;i++)
        {
            double ll = point_distance(last_sc[i], cen);
            double cl = point_distance(cur_sc[i], cen);
            nscale += cl/ll;
        }
        nscale = nscale / n;

        Mat move1 = Mat::eye(3,3,CV_64F);
        move1.at<double>(0,2) = -cen.x;
        move1.at<double>(1,2) = -cen.y;
        Mat scale = Mat::eye(3,3,CV_64F);
        scale.at<double>(0,0) = nscale;
        scale.at<double>(1,1) = nscale;
        Mat move2 = Mat::eye(3,3,CV_64F);
        move2.at<double>(0,2) = cen.x;
        move2.at<double>(1,2) = cen.y;
        //H_scale = findHomography(last_sc, cur_sc, 0);
        H_scale = move1 * scale * move2;
        return true;
    }*/
    else
    {
        return false;
    }
}

Mat ThreadCompensation::computeAffine()
{
    //LOGI("step1");
    Mat lastFrame = ThreadContext::frameVec[cm_las_index_];
    Mat frame = ThreadContext::frameVec[cm_cur_index_];
    frameSize.height=frame.rows;
    frameSize.width=frame.cols;
    LOGI("frameSize:%d, %d", frameSize.width, frameSize.height);

    //LOGI("step2");
    curGray = frame.rowRange(0,frame.rows * 2 / 3);
    LOGE("curGray size: %d, %d", curGray.cols, curGray.rows);
    resize(curGray, curGray, cv::Size(curGray.cols / ThreadContext::DOWNSAMPLE_SCALE, curGray.rows / ThreadContext::DOWNSAMPLE_SCALE));

    if (ex_index_ == 0) {
        //LOGI("step3");
        lastGray = lastFrame.rowRange(0,lastFrame.rows * 2 / 3);
        resize(lastGray, lastGray, cv::Size(lastGray.cols / ThreadContext::DOWNSAMPLE_SCALE, lastGray.rows / ThreadContext::DOWNSAMPLE_SCALE));
        detect_feature();
    }

    //特征点中心化
    track_feature();


    if(is_first_use_rtheta){
        ThreadContext::rTheta.pop();
        is_first_use_rtheta = false;
    }
    Vec<double, 3> rot = ThreadContext::rTheta.front();//前一帧的旋转矩阵
    ThreadContext::rTheta.pop();
    Vec<double, 3> er = lastRot-rot;
    lastRot = rot;
    //LOGI("see r : %f, %f, %f ", er[0], er[1], er[2]);
    double error = er[0]*er[0] + er[1]*er[1] + er[2]*er[2];


    bool sc = false;
    sc = stable_count(error);
    is_stable_ = sc;
    LOGI("see error : %f, %d ", error, is_stable_);

    //LOGI("step5");
    Mat aff;
    if(sc)
    {
//        aff = H_scale.clone();
        aff = RR2stableVec * stableVec2RR;
    }
    else
    {
        aff = calHomography();
    }

    //LOGI("step6");
    lastFeatures.clear();
    lastFeatures.assign(curFeatures.begin(), curFeatures.end());

    curGray.copyTo(lastGray);

//    for(auto p : curFeaturesTmp){
//        cv::circle(ThreadContext::frameVec[cm_cur_index_], p, 3, cv::Scalar(255, 0, 0),5);
//    }
    return aff;
}

void ThreadCompensation::frameCompensate()
{
    /*为旋转插值准备数据，即计算仿射矩阵，计算本段非关键帧到前关键帧的仿射矩阵与前关键帧到后关键帧的仿射矩阵*/
    //LOGI("cal aff");
    Mat aff = computeAffine();

    cv::Mat old_r_mat = threads::ThreadContext::r_convert_que.front();
    threads::ThreadContext::r_convert_que.pop();
    auto new_aff = aff;

    cv::Mat r_temp;
    if(!is_stable_ ){
        cv::Mat temp =  old_r_mat * threads::ThreadContext::last_old_Rotation_.inv();
        auto T = CalTranslationByR(temp);
        LOGI("11111tx:%f, ty:%f， %f, %f", T[0], T[1], aff.at<double>(0, 2), aff.at<double>(1, 2));
        new_aff = aff;
        new_aff.at<double>(0, 2) -= (new_aff.at<double>(0, 0) * T[0]);
        new_aff.at<double>(1, 2) -= (new_aff.at<double>(1, 1) * T[1]);
        r_temp = inmat * old_r_mat * threads::ThreadContext::last_old_Rotation_.inv() * inmat.inv();
    } else {
        r_temp = cv::Mat::eye(3, 3, CV_64F);
    }
    threads::ThreadContext::last_old_Rotation_ = old_r_mat;

    new_aff = r_temp * new_aff;
    trans_que.push(new_aff);
    bool readyToPull = filter.push(new_aff.clone());
    if (readyToPull) {
        cv::Mat gooda = filter.pop();

//        WriteToFile(file_before, file_after, gooda, frame_count, old_trans_mat);
        cv::Mat goodar = gooda * ThreadContext::stableRVec[out_index_];


        ////*************测试***********************////
        cv::Mat new_r_mat = threads::ThreadContext::r_convert_new_que.front();
        threads::ThreadContext::r_convert_new_que.pop();
        ////*************测试***********************////
//        cv::Mat goodar = ThreadContext::stableRVec[out_index_];
//        cv::Mat goodar = gooda;

        goodar = gooda;
//        goodar = ThreadContext::stableRVec[out_index_];

        if( cropControlFlag )
        {
            cropControl(cropRation, frameSize, goodar);

            cv::Mat scale = cv::Mat::eye(3,3,CV_64F);
            cv::Mat move = cv::Mat::eye(3,3,CV_64F);
            double croph=frameSize.height*cropRation;
            double cropw=frameSize.width*cropRation;
            double mh=(frameSize.height-croph)/2;
            double mw=(frameSize.width-cropw)/2;
            scale.at<double>(0,0) = 1.0 / cropRation;
            scale.at<double>(1,1) = 1.0 / cropRation;
            move.at<double>(0,2) = -mw;
            move.at<double>(1,2) = -mh;

            goodar = scale * move * goodar;
        }

        goodar.copyTo(ThreadContext::stableTransformVec[out_index_]);
        out_index_ = (out_index_ + 1) % ThreadContext::BUFFERSIZE;

        frame_count++;
        ThreadRollingShutter::getStableStatus(is_stable_);
        ThreadContext::rs_semaphore_->Signal();
    }

    //LOGI("compensate end");
}

bool isInside(cv::Mat cropvertex ,cv::Mat newvertex)
{
    bool aInside = true;
    for( int i = 0 ; i < 4 ; i++ )
    {
        for( int j = 0 ; j < 4 ; j++ )
        {
            cv::Point2f vec1 , vec2;
            vec1.x=float(newvertex.at<double>(0,j)-cropvertex.at<double>(0,i));
            vec1.y=float(newvertex.at<double>(1,j)-cropvertex.at<double>(1,i));
            vec2.x=float(newvertex.at<double>(0,(j+1)%4)-newvertex.at<double>(0,j));
            vec2.y=float(newvertex.at<double>(1,(j+1)%4)-newvertex.at<double>(1,j));

            // vec1 = pt_transform[j] - pt_crop[i];
            // vec2 = pt_transform[(j+1)%4] - pt_transform[j];
            float cross_product = vec1.x * vec2.y - vec2.x * vec1.y;
            //  NSLog(@"%f",cross_product);
            if( cross_product > 0 )
            {
                aInside = false;
                break;
            }
        }
        if( !aInside )
        {
            break;
        }
    }
    return aInside;

}

//经过平移旋转后仍包括裁剪窗口返回true，否则返回false，且修改affine
bool ThreadCompensation::cropControl( float cropr , Size size , Mat &affine )
{
    bool doCrop = false;
    int out=0;
    Mat vertex=(cv::Mat_<double>(3, 4)<<0.0,0.0,size.width-1,size.width-1,0.0,size.height-1,size.height-1,0.0,1.0,1.0,1.0,1.0);
    double croph=size.height*cropr;
    double cropw=size.width*cropr;
    double mh=(size.height-croph)/2;
    double mw=(size.width-cropw)/2;
    Mat cropvertex=(cv::Mat_<double>(3, 4)<<mw,mw,cropw+mw-1,cropw+mw-1,mh,croph+mh-1,croph+mh-1,mh,1.0,1.0,1.0,1.0);

    Mat stableVec = affine.clone();
    //cout<<"crop"<<crop<<endl;
    Mat newvertex = stableVec * vertex;
    newvertex.at<double>(0, 0) = newvertex.at<double>(0, 0) / newvertex.at<double>(2, 0);
    newvertex.at<double>(1, 0) = newvertex.at<double>(1, 0) / newvertex.at<double>(2, 0);

    newvertex.at<double>(0, 1) = newvertex.at<double>(0, 1) / newvertex.at<double>(2, 1);
    newvertex.at<double>(1, 1) = newvertex.at<double>(1, 1) / newvertex.at<double>(2, 1);

    newvertex.at<double>(0, 2) = newvertex.at<double>(0, 2) / newvertex.at<double>(2, 2);
    newvertex.at<double>(1, 2) = newvertex.at<double>(1, 2) / newvertex.at<double>(2, 2);

    newvertex.at<double>(0, 3) = newvertex.at<double>(0, 3) / newvertex.at<double>(2, 3);
    newvertex.at<double>(1, 3) = newvertex.at<double>(1, 3) / newvertex.at<double>(2, 3);

    bool allInside=isInside(cropvertex,newvertex);
    double ratio=1.0;
    cv::Mat I=cv::Mat::eye(3, 3, CV_64F);
    cv::Mat resultVec=stableVec.clone();
    while((!allInside)&&(ratio>=0))
    {
        doCrop = true;
        double transdet= determinant(stableVec);//求行列式
        cv::Mat transtemp=stableVec/pow(transdet, 1.0/3);
        resultVec=I*(1-ratio)+transtemp*ratio;

        ratio=ratio-0.01;
        newvertex=resultVec*vertex;
        newvertex.at<double>(0,0)=newvertex.at<double>(0,0)/newvertex.at<double>(2,0);
        newvertex.at<double>(1,0)=newvertex.at<double>(1,0)/newvertex.at<double>(2,0);

        newvertex.at<double>(0,1)=newvertex.at<double>(0,1)/newvertex.at<double>(2,1);
        newvertex.at<double>(1,1)=newvertex.at<double>(1,1)/newvertex.at<double>(2,1);

        newvertex.at<double>(0,2)=newvertex.at<double>(0,2)/newvertex.at<double>(2,2);
        newvertex.at<double>(1,2)=newvertex.at<double>(1,2)/newvertex.at<double>(2,2);

        newvertex.at<double>(0,3)=newvertex.at<double>(0,3)/newvertex.at<double>(2,3);
        newvertex.at<double>(1,3)=newvertex.at<double>(1,3)/newvertex.at<double>(2,3);

        allInside = isInside(cropvertex,newvertex);
    }

    if(doCrop)
    {
        resultVec.copyTo(affine);
    }

    return doCrop;
}

//判断经过变换后是否依然包括裁剪窗口
bool ThreadCompensation::isInsideAfterTransform( Mat &affine , vector<Point2f> &pt_crop , vector<Point2f> &pt )
{
    vector<Point2f> pt_transform;//变换后的坐标
    for( int i = 0 ; i < 4 ; i++ )
    {
        Mat tmp = Mat::ones(3,1,CV_64FC1);
        tmp.at<double>(0,0) = pt[i].x;
        tmp.at<double>(1,0) = pt[i].y;
        Mat pos = affine * tmp;
        Point2f p;
        p.x = (float) pos.at<double>(0, 0);
        p.y = (float) pos.at<double>(1, 0);
        pt_transform.push_back(p);
    }
    //判断裁剪窗口顶点坐标是否都在变换后的坐标组成的矩形内
    bool allInside = true;
    for( int i = 0 ; i < 4 ; i++ )
    {
        for( int j = 0 ; j < 4 ; j++ )
        {
            Point2f vec1 , vec2;
            vec1 = pt_transform[j] - pt_crop[i];
            vec2 = pt_transform[(j+1)%4] - pt_transform[j];
            float cross_product = vec1.x * vec2.y - vec2.x * vec1.y;
            if( cross_product > 0 )
            {
                allInside = false;
                break;
            }
        }
        if( !allInside )
        {
            break;
        }
    }

    return allInside;
}

//计算最大旋转角
//img_line：起点为原点，与X轴重合，长度为图像长或宽
//crop_line：裁剪线
//degree：旋转角度，正值为逆时针旋转
//center：旋转中心
double ThreadCompensation::computeMaxDegree( vector<Point2f> img_line , vector<Point2f> crop_line , double degree , Point2f center )
{
    if( degree > 0 )
    {
        if( center.x <= crop_line[0].x )
        {
            return 3.1415926;
        }
        else
        {
            float dis = sqrt( pow(crop_line[0].x - center.x,2) + pow(crop_line[0].y - center.y,2) );
            if( dis <= center.y )
            {
                return 3.1415926;
            }
            else
            {
                /*计算切点*/
                float a1 , a2 , a3;
                a1 = center.x - crop_line[0].x;
                a2 = center.y - crop_line[0].y;
                a3 = center.x*crop_line[0].x - center.x*center.x + center.y*crop_line[0].y;
                float k , n;
                k = -a2 / a1;
                n = -a3 / a1;
                float a , b , c;
                a = k*k + 1;
                b = 2*k*n - 2*center.x*k - 2*center.y;
                c = n*n - 2*center.x*n + center.x*center.x;
                Point2f pointofContact;
                float y1 , y2;
                y1 = (-b + sqrt(b*b - 4*a*c)) / (2*a);
                y2 = (-b - sqrt(b*b - 4*a*c)) / (2*a);
                pointofContact.y = (y1<y2)?y1:y2;
                pointofContact.x = k*pointofContact.y + n;
                /**/

                Point2f vec1 , vec2;
                vec1 = pointofContact - crop_line[0];
                vec2 = crop_line[1] - crop_line[0];
                float cos_alpha = (vec1.x*vec2.x + vec1.y*vec2.y) / (sqrt(vec1.x*vec1.x+vec1.y*vec1.y) * sqrt(vec2.x*vec2.x+vec2.y*vec2.y));
                double alpha = acos(cos_alpha);

                return alpha;
            }
        }
    }
    else
    {
        if( center.x >= crop_line[1].x )
        {
            return -3.1415926;
        }
        else
        {
            float dis = (float) sqrt(pow(crop_line[1].x - center.x, 2) + pow(crop_line[1].y - center.y, 2) );
            if( dis <= center.y )
            {
                return -3.1415926;
            }
            else
            {
                /*计算切点*/
                float a1 , a2 , a3;
                a1 = center.x - crop_line[1].x;
                a2 = center.y - crop_line[1].y;
                a3 = center.x*crop_line[1].x - center.x*center.x + center.y*crop_line[1].y;
                float k , n;
                k = -a2 / a1;
                n = -a3 / a1;
                float a , b , c;
                a = k*k + 1;
                b = 2*k*n - 2*center.x*k - 2*center.y;
                c = n*n - 2*center.x*n + center.x*center.x;
                Point2f pointofContact;
                float y1 , y2;
                y1 = (-b + sqrt(b*b - 4*a*c)) / (2*a);
                y2 = (-b - sqrt(b*b - 4*a*c)) / (2*a);
                pointofContact.y = (y1<y2)?y1:y2;
                pointofContact.x = k*pointofContact.y + n;
                /**/

                Point2f vec1 , vec2;
                vec1 = pointofContact - crop_line[1];
                vec2 = crop_line[0] - crop_line[1];
                float cos_alpha = (vec1.x*vec2.x + vec1.y*vec2.y) / (sqrt(vec1.x*vec1.x+vec1.y*vec1.y) * sqrt(vec2.x*vec2.x+vec2.y*vec2.y));
                double alpha = acos(cos_alpha);

                return -alpha;
            }
        }
    }
}
void ThreadCompensation::WriteToFile(FILE* old_file, FILE* new_file, cv::Mat mat, int count, cv::Mat old_mat) {
    point_before = old_mat * point_before;
    cv::Mat pointafter = mat * point_before;
    LOGI("gooda:[%f, %f,%f,%f,%f,%f,%f,%f,%f]",
         mat.at<double>(0, 0), mat.at<double>(0, 1), mat.at<double>(0, 2),
         mat.at<double>(1, 0), mat.at<double>(1, 1), mat.at<double>(1, 2),
         mat.at<double>(2, 0), mat.at<double>(2, 1), mat.at<double>(2, 2));
    char content_before[60];
    char content_after[60];
    sprintf(content_before, "%d %f %f\n", frame_count, point_before.at<double>(0, 0), point_before.at<double>(1, 0));
    sprintf(content_after, "%d %f %f\n", frame_count, pointafter.at<double>(0, 0), pointafter.at<double>(1, 0));
    fwrite(content_before, sizeof(char), strlen(content_before), old_file);
    fwrite(content_after, sizeof(char), strlen(content_after), new_file);
}
cv::Vec2f ThreadCompensation::CalTranslationByR(cv::Mat r) {
    std::vector<cv::Point2f> feature = feature_by_r_.front();
    feature_by_r_.pop();
    float t_x = 0;
    float t_y = 0;
    for(auto p : feature){
        cv::Mat p1 = (cv::Mat_<double>(3, 1) << p.x, p.y, 1.0f);
        cv::Mat p2 = inmat * r * inmat.inv() * p1;

        t_x += (p2.at<double>(0, 0) - p1.at<double>(0, 0));
        t_y += (p2.at<double>(1, 0) - p1.at<double>(1, 0));
    }
    if(feature.size()){
        return cv::Vec2f(t_x/feature.size(), t_y/feature.size());
    } else {
        return cv::Vec2f(0, 0);
    }


}
cv::Mat ThreadCompensation::calHomography() {
    cv::Mat homography;
//    if(lastFeaturesTmp.size() != 0 && curFeaturesTmp.size() != 0){
//        homography = cv::findHomography(lastFeaturesTmp, curFeaturesTmp, cv::RHO);
//
//    } else {
//        LOGI("i am here!!!!");
//        homography = RR2stableVec * stableVec2RR;
//    }
//
//    if(homography.empty() || isnan(homography.at<double>(0, 0))){
//        homography = RR2stableVec * stableVec2RR;
//    }
//    LOGI("homography:%f, %f, %f", T[0].at<double>(0), T[0].at<double>(1), T[0].at<double>(2));
    LOGI("new_aff:%d, %d", lastFeaturesTmp.size(), curFeaturesTmp.size());
    if(lastFeaturesTmp.size() < 3){
        homography = RR2stableVec * stableVec2RR;
    } else {
        if (!lastFeaturesTmp.empty() && !curFeaturesTmp.empty() && lastFeaturesTmp.size() >= 15){
            homography = cv::findHomography(lastFeaturesTmp, curFeaturesTmp, cv::RHO);
        }
        else if(!lastFeaturesTmp.empty() && !curFeaturesTmp.empty() && lastFeaturesTmp.size() > 1){
            if(lastFeaturesTmp.size()==3)
            {
                LOGI("i am here!!!!");
                cv::Mat AF=cv::getAffineTransform(lastFeaturesTmp,curFeaturesTmp);
                LOGI("new_aff:%d, [%f, %f, %f, %f, %f, %f, %f, %f, %f]", curFeaturesTmp.size(),
                     AF.at<double>(0, 0), AF.at<double>(0, 1), AF.at<double>(0, 2),
                     AF.at<double>(1, 0), AF.at<double>(1, 1), AF.at<double>(1, 2),
                     AF.at<double>(2, 0), AF.at<double>(2, 1), AF.at<double>(2, 2));
                homography=cv::Mat::zeros(3,3,AF.type());
                if(!AF.empty() && !isnan(AF.at<double>(0, 0))){
                    homography.at<double>(0,0)=AF.at<double>(0,0);
                    homography.at<double>(0,1)=AF.at<double>(0,1);
                    homography.at<double>(0,2)=AF.at<double>(0,2);
                    homography.at<double>(1,0)=AF.at<double>(1,0);
                    homography.at<double>(1,1)=AF.at<double>(1,1);
                    homography.at<double>(1,2)=AF.at<double>(1,2);
                    homography.at<double>(2,0)=0;
                    homography.at<double>(2,1)=0;
                    homography.at<double>(2,2)=1;
                } else{
                    homography = RR2stableVec * stableVec2RR;
                }


            }else {
                homography = RR2stableVec * stableVec2RR;
            }
        } else {
            homography = RR2stableVec * stableVec2RR;
        }

    }


    return homography;

}
