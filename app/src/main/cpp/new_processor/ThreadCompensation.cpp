//
// Created by ShiJJ on 2020/4/18.
//

#include "ThreadCompensation.h"
#include <android/log.h>
#define LOG_TAG    "c_ThreadCompensation"
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
static std::FILE* file_old = nullptr;
static std::FILE* file_new = nullptr;
static int frame_count = 0;
ThreadCompensation::~ThreadCompensation() {
    work_thread_.join();
    fclose(file_old);
    fclose(file_new);
}
void ThreadCompensation::Start() {
    file_old = fopen("/data/data/me.zhehua.gryostable/data/before.txt", "a");
    file_new = fopen("/data/data/me.zhehua.gryostable/data/after.txt", "a");
    work_thread_ = std::thread(&ThreadCompensation::Work, this);
}
void ThreadCompensation::Work() {
    pthread_setname_np(pthread_self(), "CompensationThread");
    filiter = Filiter(5, 20);
    filiter1 = Filiter(7, 20);
    digital_filter_ = DigitalFilter(10, 20, DigitalFilter::delta_T);
    last_rot_[0] = 0;
    last_rot_[1] = 0;
    last_rot_[2] = 0;
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
    frame_size_.height = ThreadContext::frame_vec_[0].rows;
    frame_size_.width = ThreadContext::frame_vec_[0].cols;
    ////**************数字部分*******************////
    cv::Mat aff = ComputeAffine();
    bool ready_to_pull_digt = digital_filter_.push(aff);
    if(ready_to_pull_digt){
        aff_queue_.push(digital_filter_.pop());
    }
    ////**************数字部分*******************////
    Quaternion first_q(ThreadContext::quaternion_vec_[cm_index_].at<double>(0, 1),
                       ThreadContext::quaternion_vec_[cm_index_].at<double>(0, 2),
                       ThreadContext::quaternion_vec_[cm_index_].at<double>(0, 2),
                       ThreadContext::quaternion_vec_[cm_index_].at<double>(0, 3));
    //filiter process start
    bool ready_to_pull = filiter.Push(first_q);
    q_cache_.push(first_q);
    RollingShutter(cm_index_);
    if(ready_to_pull){
        Quaternion new_q = filiter.Pop();
        Quaternion convert_q;
        bool ready_to_pull1 = filiter1.Push(new_q);
        if(ready_to_pull1){
            Quaternion new_q1 = filiter1.Pop();
            bool ready_to_pull2 = filiter2.Push(new_q1);
            if(ready_to_pull2){
                Quaternion new_q2 = filiter2.Pop();
                Quaternion old_q = q_cache_.front();
                q_cache_.pop();
                convert_q = Quaternion::Q1ToQ2(old_q, new_q2);
//                WriteDataToFile(file_old, old_q, file_new, new_q2, frame_count);
                __android_log_print(ANDROID_LOG_DEBUG, "ThreadCompensation", "before:%d, %f", ++frame_count, old_q.x_);
                __android_log_print(ANDROID_LOG_DEBUG, "ThreadCompensation", "after:%d, %f", frame_count, convert_q.x_);

                cv::Mat gooda = aff_queue_.front();
                aff_queue_.pop();
                cv::Mat convert_mat = Quaternion::QuaternionToR(convert_q);
                convert_mat = ThreadContext::inmat * convert_mat * ThreadContext::inmat.inv();
                convert_mat = ThreadContext::RR2stableVec * convert_mat * ThreadContext::stableVec2RR;
                convert_mat = gooda * convert_mat;
//                convert_mat = gooda;
                //RollingShutter start
                std::vector<Quaternion> cur_frame_q = rs_q_cache_.front();
                rs_q_cache_.pop();
                cv::Mat rs_convert_mat(30, 3, CV_64F);
                int num = 0;
                for(auto it : cur_frame_q){
                    Quaternion convert =  it * cur_frame_q[0].Inv();
                    __android_log_print(ANDROID_LOG_DEBUG, "ThreadCompensation", "curframe_q:%d", cur_frame_q.size());
                    __android_log_print(ANDROID_LOG_DEBUG, "ThreadCompensation", "temp:[%lf, %lf, %lf, %lf]", convert.w_,
                                        convert.x_, convert.y_, convert.z_);
                    cv::Mat mat = Quaternion::QuaternionToR(convert);
                    mat = ThreadContext::inmat * mat * ThreadContext::inmat.inv();
                    for(int k = 0; k < mat.rows; k++){
                        for(int l = 0; l < mat.cols; l++){
                            rs_convert_mat.at<double>(k+num, l) = mat.at<double>(k, l);
                        }
                    }
                    num += 3;
                }
                rs_convert_mat.copyTo(ThreadContext::rs_convert_mat_);
                __android_log_print(ANDROID_LOG_DEBUG, "ThreadCompensation", "rs_convert_mat:%d", rs_convert_mat.rows);
                //RollingShutter end
                if(crop_control_flag){
                    CropControl(crop_ratio_, frame_size_, convert_mat);
                    cv::Mat scale = cv::Mat::eye(3,3,CV_64F);
                    cv::Mat move = cv::Mat::eye(3,3,CV_64F);
                    double croph=frame_size_.height*crop_ratio_;
                    double cropw=frame_size_.width*crop_ratio_;
                    double mh=(frame_size_.height-croph)/2;
                    double mw=(frame_size_.width-cropw)/2;
                    scale.at<double>(0,0) = 1.0 / crop_ratio_;
                    scale.at<double>(1,1) = 1.0 / crop_ratio_;
                    move.at<double>(0,2) = -mw;
                    move.at<double>(1,2) = -mh;
                    convert_mat = scale * move * convert_mat;
                }

                convert_mat.copyTo(ThreadContext::stable_vec_[out_index_]);
                ThreadContext::out_semaphore_->Singal();
                out_index_ = (out_index_ + 1) % ThreadContext::BUFFER_SIZE_;
            }
        }
    }
    //filiter process end

    cm_index_ = (cm_index_ + 1) % ThreadContext::BUFFER_SIZE_;
}
bool ThreadCompensation::CropControl(double crop_ratio, const cv::Size &size, cv::Mat &mat) {
    bool do_crop = false;
    cv::Mat vertex=(cv::Mat_<double>(3, 4)<<0.0,0.0,size.width-1,size.width-1,0.0,size.height-1,size.height-1,0.0,1.0,1.0,1.0,1.0);
    double crop_h = size.height * crop_ratio;
    double crop_w = size.width * crop_ratio;
    double mh = (size.height - crop_h) / 2;
    double mw = (size.width - crop_w) / 2;
    cv::Mat crop_vertex=(cv::Mat_<double>(3, 4)<<mw,mw,crop_w+mw-1,crop_w+mw-1,mh,crop_h+mh-1,crop_h+mh-1,mh,1.0,1.0,1.0,1.0);
    cv::Mat stable_vec = mat.clone();
    cv::Mat newvertex = stable_vec * vertex;
    newvertex.at<double>(0, 0) = newvertex.at<double>(0, 0) / newvertex.at<double>(2, 0);
    newvertex.at<double>(1, 0) = newvertex.at<double>(1, 0) / newvertex.at<double>(2, 0);

    newvertex.at<double>(0, 1) = newvertex.at<double>(0, 1) / newvertex.at<double>(2, 1);
    newvertex.at<double>(1, 1) = newvertex.at<double>(1, 1) / newvertex.at<double>(2, 1);

    newvertex.at<double>(0, 2) = newvertex.at<double>(0, 2) / newvertex.at<double>(2, 2);
    newvertex.at<double>(1, 2) = newvertex.at<double>(1, 2) / newvertex.at<double>(2, 2);

    newvertex.at<double>(0, 3) = newvertex.at<double>(0, 3) / newvertex.at<double>(2, 3);
    newvertex.at<double>(1, 3) = newvertex.at<double>(1, 3) / newvertex.at<double>(2, 3);
    bool all_inside = IsInside(crop_vertex, newvertex);
    double ratio = 1.0;
    cv::Mat I=cv::Mat::eye(3, 3, CV_64F);
    cv::Mat result_vec = stable_vec.clone();
    while ((!all_inside) && (ratio >= 0)){
        do_crop = true;
        double transdet = cv::determinant(stable_vec);
        cv::Mat trans_temp = stable_vec / pow(transdet, 1.0/3);
        result_vec = I * (1 - ratio) + trans_temp * ratio;
        ratio = ratio - 0.01;
        newvertex = result_vec * vertex;
        newvertex.at<double>(0,0)=newvertex.at<double>(0,0)/newvertex.at<double>(2,0);
        newvertex.at<double>(1,0)=newvertex.at<double>(1,0)/newvertex.at<double>(2,0);

        newvertex.at<double>(0,1)=newvertex.at<double>(0,1)/newvertex.at<double>(2,1);
        newvertex.at<double>(1,1)=newvertex.at<double>(1,1)/newvertex.at<double>(2,1);

        newvertex.at<double>(0,2)=newvertex.at<double>(0,2)/newvertex.at<double>(2,2);
        newvertex.at<double>(1,2)=newvertex.at<double>(1,2)/newvertex.at<double>(2,2);

        newvertex.at<double>(0,3)=newvertex.at<double>(0,3)/newvertex.at<double>(2,3);
        newvertex.at<double>(1,3)=newvertex.at<double>(1,3)/newvertex.at<double>(2,3);
        all_inside = IsInside(crop_vertex, newvertex);
    }
    if(do_crop){
        result_vec.copyTo(mat);
    }
    return do_crop;
}
bool ThreadCompensation::IsInside(cv::Mat crop_vertex ,cv::Mat new_vertex)
{
    bool aInside = true;
    for( int i = 0 ; i < 4 ; i++ )
    {
        for( int j = 0 ; j < 4 ; j++ )
        {
            cv::Point2f vec1 , vec2;
            vec1.x=float(new_vertex.at<double>(0,j)-crop_vertex.at<double>(0,i));
            vec1.y=float(new_vertex.at<double>(1,j)-crop_vertex.at<double>(1,i));
            vec2.x=float(new_vertex.at<double>(0,(j+1)%4)-new_vertex.at<double>(0,j));
            vec2.y=float(new_vertex.at<double>(1,(j+1)%4)-new_vertex.at<double>(1,j));

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
void ThreadCompensation::RollingShutter(int cm_index) {
    std::vector<Quaternion> y;
    std::vector<double> x;
    std::vector<double> x0;
    cv::Mat mat = ThreadContext::quaternion_vec_[cm_index];
    for(int i = 0; i < mat.rows; i++){
        x.push_back(mat.at<double>(i, 0));
        y.emplace_back(Quaternion(ThreadContext::quaternion_vec_[cm_index_].at<double>(i, 1),
                ThreadContext::quaternion_vec_[cm_index_].at<double>(i, 2),
                ThreadContext::quaternion_vec_[cm_index_].at<double>(i, 3),
                ThreadContext::quaternion_vec_[cm_index_].at<double>(i, 4)));
    }
    x0 = GetTimeStampInFrame(x[0], x[x.size() - 1], 10);
    std::vector<Quaternion> out = Quaternion::Interpolation(x0, x, y);
    rs_q_cache_.push(out);
}
std::vector<double> ThreadCompensation::GetTimeStampInFrame(double timestart, double timeend,
                                                         int num) {
    std::vector<double> timeStampInFrame;
    double space = (timeend-timestart)/(num-1);
    for(int i = 0; i < num; i++){
        double temp = timestart+i*space;
        timeStampInFrame.push_back(temp);

    }
    return timeStampInFrame;
}
void ThreadCompensation::WriteDataToFile(std::FILE *file_old, const Quaternion &old_q,
                                         std::FILE *file_new, const Quaternion &new_q, int frame) {
    char before[60];
    char after[60];
    sprintf(before, "before %d %f %f %f %f\n", frame, old_q.w_, old_q.x_, old_q.y_, old_q.z_);
    sprintf(after, "after %d %f %f %f %f\n", frame, new_q.w_, new_q.x_, new_q.y_, new_q.z_);
    fwrite(before, sizeof(char), strlen(before), file_old);
    fwrite(after, sizeof(char), strlen(after), file_new);
}
////*****************数字部分**********************////
bool ThreadCompensation::OutOfImg(const cv::Point2f &point, const cv::Size &size) {
    return (point.x <= 0 || point.y <= 0 || point.x >= size.width - 1 || point.y >= size.height - 1 );
}
double ThreadCompensation::PointDistance(const cv::Point2f &p1, const cv::Point2f &p2) {
    cv::Point2f d = p1 - p2;
    double d_mu;
    d_mu = sqrt(d.x * d.x + d.y * d.y);
    return d_mu;
}
cv::Mat ThreadCompensation::ComputeAffine() {
    //LOGI("step1");
    cv::Mat lastFrame = ThreadContext::frame_vec_[cm_las_index_];
    cv::Mat frame = ThreadContext::frame_vec_[cm_cur_index_];
    frame_size_.height=frame.rows;
    frame_size_.width=frame.cols;

    //LOGI("step2");
    cur_gray_ = frame.rowRange(0,frame.rows * 2 / 3);
    LOGE("curGray size: %d, %d", cur_gray_.cols, cur_gray_.rows);
    resize(cur_gray_, cur_gray_, cv::Size(cur_gray_.cols / ThreadContext::DOWNSAMPLE_SCALE, cur_gray_.rows / ThreadContext::DOWNSAMPLE_SCALE));

    if (ex_index_ == 0) {
        //LOGI("step3");
        last_gray_ = lastFrame.rowRange(0,lastFrame.rows * 2 / 3);
        resize(last_gray_, last_gray_, cv::Size(last_gray_.cols / ThreadContext::DOWNSAMPLE_SCALE, last_gray_.rows / ThreadContext::DOWNSAMPLE_SCALE));
        DetectFeature();
    }

    //LOGI("step4");
    TrackFeature();

//    //LOGI("see error : %f", error);
    if(is_first_use_rtheta_){
        ThreadContext::rTheta.pop();
        is_first_use_rtheta_ = false;
    }
    cv::Vec<double, 3> rot = ThreadContext::rTheta.front();//前一帧的旋转矩阵
    ThreadContext::rTheta.pop();
    cv::Vec<double, 3> er = last_rot_-rot;
    last_rot_ = rot;
    //LOGI("see r : %f, %f, %f ", er[0], er[1], er[2]);
    double error = er[0]*er[0] + er[1]*er[1] + er[2]*er[2];


    bool sc = false;
    sc = StableCount(error);

    //LOGI("step5");
    cv::Mat aff;
    LOGI("see error : %d ", sc);
    if(sc)
    {
        aff = h_scale_.clone();
    }
    else
    {
        aff = CalculHomo(1);
    }

    //LOGI("step6");
    last_features_.clear();
    last_features_.assign(cur_features_.begin(), cur_features_.end());
    cur_gray_.copyTo(last_gray_);

    return aff;
}
void ThreadCompensation::DetectFeature() {
    double quality_level = 0.1;
    double min_distance = 8;
    int max_corners = 8;

    std::vector<cv::Point2f> lastFeatures_a[16], startp;
    last_features_.clear();

    int half_w=last_gray_.cols/4 , half_h=last_gray_.rows/4;
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
        lastGray_a[i] = last_gray_(rect[i]);
        goodFeaturesToTrack(lastGray_a[i], lastFeatures_a[i], max_corners, quality_level, min_distance);//检测特征点
        for(int j=0; j<lastFeatures_a[i].size(); j++)
        {
            cv::Point2f pt=lastFeatures_a[i][j]+startp[i];//转化成全局坐标
            last_features_.push_back(pt);
        }
    }
}
void ThreadCompensation::TrackFeature() {
    double rate = 1.4;

    status_.clear();
    std::vector<float> err;
    cur_features_.clear();

    //LOGI("step4_1");
    calcOpticalFlowPyrLK( last_gray_ , cur_gray_ , last_features_ , cur_features_ , status_ , err);//根据已检测到的前一帧特征点在后一帧查找匹配的特征点
    //如果没找到匹配点会将前一帧特征点位置复制到curFeatures中，并在status中标记为0
    status_choose_.clear();
    status_choose_.assign(status_.begin(), status_.end());//将status复制到status_choose中

    //LOGI("step4_2");
    int max = last_features_.size() < cur_features_.size() ? last_features_.size() : cur_features_.size();
    double dis_sum=0;
    //如果是没有找到匹配点，两点之间的距离为0，不影响平均距离
    for(int i=0;i<max;i++)
    {
        dis_sum += PointDistance(last_features_[i],cur_features_[i]);
    }
    double dis_avg=0;
    dis_avg=dis_sum/max;

    //LOGI("step4_3");
    max = max < status_.size() ? max : status_.size();
    for(int i=0;i<max;i++)
    {
        if(PointDistance(last_features_[i],cur_features_[i]) > dis_avg * rate)
        {
            status_[i] = 0;
        }
    }//如果大于特征点之间的距离大于平均距离的1.4倍，则舍弃

    //LOGI("step4_4");
    cv::Mat m_Fundamental;
    std::vector<uchar> m_RANSACStatus;
    cv::Mat p1(last_features_);
    cv::Mat p2(cur_features_);
    double outliner=0;
    m_Fundamental = findFundamentalMat(p1, p2, m_RANSACStatus, cv::FM_RANSAC, 3, 0.99);

    //LOGI("step4_5");
    for (int j = 0 ; j < status_.size() ; j++ )
    {
        if (m_RANSACStatus.size() > j && m_RANSACStatus[j] == 0) // 状态为0表示野点(误匹配)
        {
            status_[j] = 0;
        }
        if(status_[j]==0)
        {
            outliner++;
        }
    }

    int idx = 0;
    cur_features_tmp_.clear();
    last_features_tmp_.clear();

    //LOGI("step4_6");
    for (auto itC = cur_features_.begin(), itP = last_features_.begin(); itC != cur_features_.end(); itC ++, itP ++, idx ++) {

        if (status_[idx] == 0 || err[idx] > 20 || OutOfImg(*itC, cv::Size(last_gray_.cols, last_gray_.rows))) {
            status_choose_[idx]=0;
        } else {
            cv::Point2f cfp=*itC * ThreadContext::DOWNSAMPLE_SCALE;
            cv::Point2f lfp=*itP * ThreadContext::DOWNSAMPLE_SCALE;
            cur_features_tmp_.push_back(cfp);
            last_features_tmp_.push_back(lfp);
        }
    }
}
bool ThreadCompensation::StableCount(double e) {
    double height = cur_gray_.rows * ThreadContext::DOWNSAMPLE_SCALE;
    double width = cur_gray_.cols * ThreadContext::DOWNSAMPLE_SCALE;
    cv::Point2f cen(width/2, height/2);
    double long_side = width > height ? width : height;
    double limit_cor = sqrt(pow(height, 2) + pow(width, 2))/15;

    double sta_sca_limit = 0.00015;
    double sta_limit = long_side * sta_sca_limit;
    double sca_limit = long_side * sta_sca_limit * 4;
    int numStable = 0,numScale = 0;

    std::vector<cv::Point2f> last_sc,cur_sc;

    int num=(cur_features_.size()<last_features_.size()?cur_features_.size():last_features_.size());
    for(int i=0;i<num;i++)
    {
        cv::Point2f d = cur_features_[i] - last_features_[i];
        float d_mu;
        d_mu = sqrt(d.x * d.x + d.y * d.y);
        if (d_mu > sca_limit || status_choose_[i]==0)
        {

        }
        else if (d_mu > sta_limit && d_mu <= sca_limit)
        {
            cv::Point2f cp1 = cur_features_[i] * ThreadContext::DOWNSAMPLE_SCALE, lp1 = last_features_[i] * ThreadContext::DOWNSAMPLE_SCALE;
            double d1 = LineDistance(cen,cp1,lp1);
            if(d1 < limit_cor)
            {
                cur_sc.push_back(cp1);
                last_sc.push_back(lp1);
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
        h_scale_ = cv::Mat::eye(3, 3, CV_64F);
        return true;
    }
    else
    {
        return false;
    }
}
double ThreadCompensation::LineDistance(cv::Point2f p0, cv::Point2f p1, cv::Point2f p2) {
    double d = (fabs((p2.y - p1.y) * p0.x +(p1.x - p2.x) * p0.y + ((p2.x * p1.y) -(p1.x * p2.y)))) / (sqrt(pow(p2.y - p1.y, 2) + pow(p1.x - p2.x, 2)));
    return d;
}
cv::Mat ThreadCompensation::CalculHomo(int niter) {
    cv::Mat H = cv::Mat();
    std::vector<char> ifselect;
    if(last_features_tmp_.size() < 3)
    {
        H = cv::Mat::eye(3, 3, CV_64F);
    }
    else
    {
//        std::vector<cv::Point2f> p1,p2;
//        if(with_roll_)
//        {
//            bool ok = affPointSimplify_tri(p1,p2);
//            if(ok)
//            {
//                H = getAffineTransform(p1 , p2);
//            } else{
//                H = cv::Mat::eye(3, 3, CV_64F);
//            }
//        } else{
            H = MoveAndScale();
//        }
    }

    cv::Mat aff = cv::Mat::zeros(3,3,CV_64FC1);
    aff.at<double>(0,0) = H.at<double>(0,0);
    aff.at<double>(0,1) = H.at<double>(0,1);
    aff.at<double>(0,2) = H.at<double>(0,2);
    aff.at<double>(1,0) = H.at<double>(1,0);
    aff.at<double>(1,1) = H.at<double>(1,1);
    aff.at<double>(1,2) = H.at<double>(1,2);
    aff.at<double>(2,2) = 1;

    return aff;
}
cv::Mat ThreadCompensation::MoveAndScale() {
    double hw = last_gray_.cols * ThreadContext::DOWNSAMPLE_SCALE / 2;
    double hh = last_gray_.rows * ThreadContext::DOWNSAMPLE_SCALE / 2;
    int poi_count[3];
    cv::Point2f last_poi_avg[3], cur_poi_avg[3];

    for(int i=0; i<3; i++)
    {
        last_poi_avg[i] = cv::Point2f(0,0);
        cur_poi_avg[i] = cv::Point2f(0,0);
        poi_count[i] = 0;
    }

    int n = last_features_tmp_.size() < cur_features_tmp_.size() ? last_features_tmp_.size() : cur_features_tmp_.size();
    for(int i=0; i<n; i++)
    {
        if(last_features_tmp_[i].y < hh)
        {
            poi_count[0]++;
            last_poi_avg[0] += last_features_tmp_[i];
            cur_poi_avg[0] += cur_features_tmp_[i];
        }
        else
        {
            poi_count[1]++;
            last_poi_avg[1] += last_features_tmp_[i];
            cur_poi_avg[1] += cur_features_tmp_[i];
        }

        poi_count[2]++;
        last_poi_avg[2] += last_features_tmp_[i];
        cur_poi_avg[2] += cur_features_tmp_[i];
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

    cv::Mat move = cv::Mat::eye(3,3,CV_64F);
    if(!all_ok)
    {
        return move;
    }
    cv::Mat scale = cv::Mat::eye(3,3,CV_64F);
    cv::Point2f m = cur_poi_avg[2] - last_poi_avg[2];
    double l[3];
    l[0] = PointDistance(last_poi_avg[0], last_poi_avg[1]);//前一帧上半部分和下半部分特征点平均值之间的距离
    l[1] = PointDistance(cur_poi_avg[0], cur_poi_avg[1]);//后一帧上半部分和下半部分特征点平均值之间的距离
    l[2] = l[1]/l[0];//距离之比
    LOGI("distance:%f, %f", m.x, m.y);
    scale.at<double>(0,0) = l[2];
    scale.at<double>(1,1) = l[2];
    move.at<double>(0,2) = m.x;
    move.at<double>(1,2) = m.y;
    return (scale * move);
}