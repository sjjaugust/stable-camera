//
// Created by ShiJJ on 2020/6/15.
//

#include "AutoFilter.h"
#include "BSpline.h"
#define LOG_TAG    "c_ThreadCompensation"
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

static std::ofstream file_w("window.txt");
static int frame_count = 1;
static std::ofstream file_w1("middle.txt");
static int frame_count_m = 1;
static bool is_first = true;
int AutoFilter::predict_num_ = 5;
AutoFilter::AutoFilter(int max_size, double sigma)
:max_size_(max_size), sigma_(sigma)
{
    std::cout << max_size_ <<" " <<sigma_ << std::endl;
    weight_vec_.clear();
    weight_vec_.resize(max_size_);
    for (int i = 0; i < max_size_; i ++) {
        double k=i-delay_num_;
        double weight = filterWeight(k, sigma);
        weight_vec_[i] = weight;
    }

    size_ = cv::Size(1920, 1080);
    vertex_ = (cv::Mat_<double>(3, 4)<<0.0,0.0,size_.width-1,size_.width-1,0.0,size_.height-1,size_.height-1,0.0,1.0,1.0,1.0,1.0);
    double cropw = crop_rate_ * size_.width;
    double croph = crop_rate_ * size_.height;
    double mw = (size_.width - cropw) / 2;
    double mh = (size_.height - croph) / 2;
    cropvertex_ = (cv::Mat_<double>(3, 4)<<mw,mw,cropw+mw,cropw+mw,mh,croph+mh,croph+mh,mh,1.0,1.0,1.0,1.0);

    for(int i=0; i<predict_num_; i++)
    {
        f_num_[i] = i+1;
    }
    num_que_ = 0;
}

bool AutoFilter::push(cv::Mat goodar) {
    if (goodar.empty()) {
        int target=max_size_/2;
        while (input_buffer_.size()>max_size_/2) {
            putIntoWindow(target);
            input_buffer_.pop_front();
        }
        return true;
    }
    input_buffer_.push_back(goodar);
    if(input_buffer_.size() < max_size_){
        if(input_buffer_.size() >= delay_num_){
            int target = input_buffer_.size() - (delay_num_);
            int offset = max_size_ - input_buffer_.size();
            if(putIntoWindow(target, offset)){
                return true;
            } else {
                return false;
            }

        } else{
            return false;
        }
    } else {
        int target = input_buffer_.size() - delay_num_;
        if(putIntoWindow(target)){
            input_buffer_.pop_front();
            return true;
        } else {
            input_buffer_.pop_front();
            return false;
        }

    }

}

cv::Mat AutoFilter::pop() {
    if(!output_buffer_.empty()){
        cv::Mat ret_mat = output_buffer_.front().clone();
        output_buffer_.pop();
        return ret_mat;
    }
    return cv::Mat();

}

bool AutoFilter::putIntoWindow(int target, int offset) {
    std::cout << "target:" << target << std::endl;
    window_.clear();
    window_.resize(input_buffer_.size());
    window_[target] = cv::Mat::eye(3, 3, CV_64F);
    for(int i = target - 1; i >= 0; i--){
        window_[i] = input_buffer_[i].inv() * window_[i + 1];
    }
    for(int i = target + 1; i < input_buffer_.size(); i++){
        window_[i] = input_buffer_[i - 1] * window_[i - 1];
    }
    double sum_weight = 0;
    cv::Mat ret_mat = cv::Mat::zeros(3, 3, CV_64F);
    for(int i = 0; i < window_.size(); i++) {
        ret_mat += (weight_vec_[i + offset] * window_[i]);
        sum_weight += weight_vec_[i + offset];
    }
    ret_mat /= sum_weight;
    processCrop(ret_mat, size_);
    if(ex_count < predict_num_){
        ex_count++;
        return false;
    }
    return true;

}


bool AutoFilter::isInside(cv::Mat cropvertex, cv::Mat newvertex) {
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

void AutoFilter::processCrop(const cv::Mat &comp, const cv::Size &size) {
    s_mat_.push(comp.clone());
    cv::Point2d crop_window[4];
    cv::Mat stable_mat = comp.clone();
    cv::Mat box = stable_mat.inv() * cropvertex_;
    crop_window[0].x=box.at<double>(0,0)/box.at<double>(2,0);
    crop_window[0].y=box.at<double>(1,0)/box.at<double>(2,0);

    crop_window[1].x=box.at<double>(0,1)/box.at<double>(2,1);
    crop_window[1].y=box.at<double>(1,1)/box.at<double>(2,1);

    crop_window[2].x=box.at<double>(0,2)/box.at<double>(2,2);
    crop_window[2].y=box.at<double>(1,2)/box.at<double>(2,2);

    crop_window[3].x=box.at<double>(0,3)/box.at<double>(2,3);
    crop_window[3].y=box.at<double>(1,3)/box.at<double>(2,3);

    double xmin = 0 - crop_window[0].x ;
    double xmax = size.width - crop_window[0].x ;
    double ymin = 0 - crop_window[0].y ;
    double ymax = size.height - crop_window[0].y ;

    for(int j=1; j<4; j++)
    {
        if(xmin < 0 - crop_window[j].x)
            xmin = 0 - crop_window[j].x;
        if(xmax > size.width - crop_window[j].x)
            xmax = size.width - crop_window[j].x;
        if(ymin < 0 - crop_window[j].y)
            ymin = 0 - crop_window[j].y;
        if(ymax > size.height - crop_window[j].y)
            ymax = size.height - crop_window[j].y;
    }
    limit temp_limit;
    temp_limit.xmin = xmin;
    temp_limit.xmax = xmax;
    temp_limit.ymin = ymin;
    temp_limit.ymax = ymax;
    limit_que_.push(temp_limit);
    LOGI("temp_limit:%d, %f, %f, %f, %f", frame_count, temp_limit.xmin, temp_limit.xmax,
         temp_limit.ymin, temp_limit.ymax);

    double trans_x[predict_num_/2], trans_y[predict_num_/2];
    double x_m, y_m;

    if(num_que_ < predict_num_) {
        num_que_++;
    }
    x_m = cur_x;
    y_m = cur_y;
    double x_d = xmax - xmin;
    double y_d = ymax - ymin;
    if(x_m < xmin){
        x_m = xmin + x_d /4;
        cur_x = x_m;
    } else if(x_m > xmax){
        x_m = xmax - x_d / 4;
        cur_x = x_m;
    }
    if(y_m < ymin){
        y_m = ymin + y_d /4;
        cur_y = y_m;
    } else if(y_m > ymax){
        y_m = ymax - y_d/4;
        cur_y = y_m;
    }
    file_w << frame_count <<" " << xmin << " " << xmax << " " << ymin << " " << ymax << " "
    << x_m << " " << y_m <<std::endl;
    frame_count++;
    queue_in(que_x_, num_que_ - 1, x_m);
    queue_in(que_y_, num_que_ - 1, y_m);
    if(num_que_ < predict_num_){
        return ;
    } else {
        if(is_first){
            cv::Mat temp = s_mat_.front().inv();
            s_mat_.pop();
            output_buffer_.push(temp);
            limit_que_.pop();
            is_first = false;
        }
        double result_x_3[predict_num_/2], result_y_3[predict_num_/2];
        Bspline bspline_x, bspline_y;
        for(int i = 0; i < predict_num_; i+=2){
            bspline_x.push(cv::Point2d(index_, que_x_[i]));
            bspline_y.push(cv::Point2d(index_, que_y_[i]));
            index_ += 2;
        }
        bspline_x.calControlPoint();
        bspline_y.calControlPoint();
        double ratio = 0.25;
        for(int i = 0; i < predict_num_/2; i++){
            result_x_3[i] = bspline_x.genInterpolationPoint(ratio).y;
            result_y_3[i] = bspline_y.genInterpolationPoint(ratio).y;
            ratio += 0.25;
        }
        for(int i = 0; i < predict_num_/2; i++){
            trans_x[i] = result_x_3[i];
            trans_y[i] = result_y_3[i];
        }
//        file_w << trans_x << " " << trans_y << std::endl;
        for(int i = 0; i < predict_num_/2; i++){
            cv::Mat comp2 = cv::Mat::eye(3, 3, CV_64F);

            limit li = limit_que_.front();
            limit_que_.pop();
            if(trans_x[i] < li.xmin){
                trans_x[i] = li.xmin;
            }else if(trans_x[i] > li.xmax){
                trans_x[i] = li.xmax;
            }
            if(trans_y[i] < li.ymin){
                trans_y[i] = li.ymin;
            }else if(trans_y[i] > li.ymax){
                trans_y[i] = li.ymax;
            }
            bool flag = false;
            if(li.xmin > li.xmax){
                LOGI("xmin is bigger than xmax!");
                flag = true;
            }
            if(li.ymin > li.ymax){
                LOGI("ymin is bigger than ymax!");
                flag = true;
            }

            comp2.at<double>(0, 2) = trans_x[i];
            comp2.at<double>(1, 2) = trans_y[i];
            cv::Mat temp = s_mat_.front().inv();
            s_mat_.pop();
            cv::Mat news = (comp2 * temp).inv();
            if(flag){
                cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
                LOGI("not compensation!");
                output_buffer_.push(I.clone());
            } else {
                output_buffer_.push(news.clone());
            }
            num_que_ = predict_num_/2 + 1;
            file_w1 << frame_count_m << " " << trans_x[i] << " " << trans_y[i] << std::endl;
            frame_count_m++;
        }

    }

}

double AutoFilter::calError(double *ori, double *aft, int n) {
    double err = 0;
    for(int i = 0; i < n; i++){
        err += abs(sqrt(abs(pow(ori[i], 2) - pow(aft[i], 2))));
    }
    return err;
}

void AutoFilter::queue_in(double *q, int m, double x) {
    for(int i=1; i<=m; i++)
    {
        q[i-1] = q[i];
    }
    q[m]=x;
}


void AutoFilter::polyfit(double *arrX, double *arrY, int num, int n, double* result) {
    int size = num;//vec里的个数
    int x_num = n + 1;//次数+1
    //构造矩阵U和Y
    cv::Mat mat_u(size, x_num, CV_64F);//num行，n+1列
    cv::Mat mat_y(size, 1, CV_64F);//num行，1列

    for (int i = 0; i < mat_u.rows; ++i)
        for (int j = 0; j < mat_u.cols; ++j)
        {
            mat_u.at<double>(i, j) = pow(arrX[i], j);//x^j次方
        }

    for (int i = 0; i < mat_y.rows; ++i)
    {
        mat_y.at<double>(i, 0) = arrY[i];
    }

    //矩阵运算，获得系数矩阵K
    cv::Mat mat_k(x_num, 1, CV_64F);
    mat_k = (mat_u.t()*mat_u).inv()*mat_u.t()*mat_y;
    //std::cout << mat_k << std::endl;
    //return mat_k;

    for(int i = 0; i < 3; i++){
        result[i] = 0;
    }
    for(int i = 0; i < 3; i++){
        for (int j = 0; j < n + 1; ++j)
        {
            result[i] += mat_k.at<double>(j, 0)*pow(i+1,j);
        }
    }

}