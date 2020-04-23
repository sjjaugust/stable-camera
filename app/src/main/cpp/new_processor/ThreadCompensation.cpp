//
// Created by ShiJJ on 2020/4/18.
//

#include "ThreadCompensation.h"
#include <android/log.h>
static int frame_count = 0;

ThreadCompensation::~ThreadCompensation() {
    work_thread_.join();
}
void ThreadCompensation::Start() {
    work_thread_ = std::thread(&ThreadCompensation::Work, this);
}
void ThreadCompensation::Work() {
    pthread_setname_np(pthread_self(), "CompensationThread");
    filiter = Filiter(20, 5, Filiter::Method1);
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
    Quaternion first_q(ThreadContext::quaternion_vec_[cm_index_].at<double>(0, 1),
                       ThreadContext::quaternion_vec_[cm_index_].at<double>(0, 2),
                       ThreadContext::quaternion_vec_[cm_index_].at<double>(0, 2),
                       ThreadContext::quaternion_vec_[cm_index_].at<double>(0, 3));
    bool ready_to_pull = filiter.Push(first_q);
    q_cache_.push(first_q);
    RollingShutter(cm_index_);
    if(ready_to_pull){
        Quaternion new_q = filiter.Pop();
        Quaternion old_q = q_cache_.front();
        q_cache_.pop();
        __android_log_print(ANDROID_LOG_DEBUG, "ThreadCompensation", "before:%d, %f", ++frame_count, old_q.x_);
        __android_log_print(ANDROID_LOG_DEBUG, "ThreadCompensation", "after:%d, %f", frame_count, new_q.x_);
//        Quaternion convert_q = Quaternion::Q1ToQ2(old_q, new_q);
        Quaternion convert_q = new_q;
        cv::Mat convert_mat = Quaternion::QuaternionToR(convert_q);
        convert_mat = ThreadContext::inmat * convert_mat * ThreadContext::inmat.inv();
        convert_mat = ThreadContext::RR2stableVec * convert_mat * ThreadContext::stableVec2RR;
        //RollingShutter start
        std::vector<Quaternion> cur_frame_q = rs_q_cache_.front();
        rs_q_cache_.pop();
        cv::Mat rs_convert_mat(30, 3, CV_64F);
        int num = 0;
        for(auto it : cur_frame_q){
            Quaternion convert = Quaternion::Q1ToQ2(it, old_q);
            __android_log_print(ANDROID_LOG_DEBUG, "ThreadCompensation", "curframe_q:%d", cur_frame_q.size());
            __android_log_print(ANDROID_LOG_DEBUG, "ThreadCompensation", "%dtemp:[%lf, %lf, %lf, %lf]", cur_frame_q[0].w_,
                                cur_frame_q[0].x_, cur_frame_q[0].y_, cur_frame_q[0].z_);
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
        y.emplace_back(Quaternion(ThreadContext::quaternion_vec_[cm_index_].at<double>(0, 1),
                ThreadContext::quaternion_vec_[cm_index_].at<double>(0, 2),
                ThreadContext::quaternion_vec_[cm_index_].at<double>(0, 2),
                ThreadContext::quaternion_vec_[cm_index_].at<double>(0, 3)));
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