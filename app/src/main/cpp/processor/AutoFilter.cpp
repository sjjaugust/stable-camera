//
// Created by ShiJJ on 2020/6/15.
//

#include "AutoFilter.h"
static std::ofstream file_w("window.txt");
static int frame_count = 1;

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

    for(int i=0; i<50; i++)
    {
        f_num[i] = i+1;
    }
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
            putIntoWindow(target, offset);
            return true;
        } else{
            return false;
        }
    } else {
        int target = input_buffer_.size() - delay_num_;
        putIntoWindow(target);
        input_buffer_.pop_front();
        return true;
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

void AutoFilter::putIntoWindow(int target, int offset) {
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
//    cv::Mat glo_t = processGlobalTrans(ret_mat, target, offset);
//    std::cout << "glot:" << glo_t << std::endl;
//    cv::Mat new_gooda = glo_t * ret_mat;
//    new_gooda = ret_mat;
    cv::Mat new_gooda = processTrans(ret_mat, size_);
    frame_count++;
    output_buffer_.push(new_gooda.clone());

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

cv::Mat AutoFilter::calGlobalTrans(const cv::Mat &comp, double crop_rate, const cv::Size& videoSize) {
    cv::Mat newvertex = comp * vertex_;
    newvertex.at<double>(0, 0) = newvertex.at<double>(0, 0) / newvertex.at<double>(2, 0);
    newvertex.at<double>(1, 0) = newvertex.at<double>(1, 0) / newvertex.at<double>(2, 0);

    newvertex.at<double>(0, 1) = newvertex.at<double>(0, 1) / newvertex.at<double>(2, 1);
    newvertex.at<double>(1, 1) = newvertex.at<double>(1, 1) / newvertex.at<double>(2, 1);

    newvertex.at<double>(0, 2) = newvertex.at<double>(0, 2) / newvertex.at<double>(2, 2);
    newvertex.at<double>(1, 2) = newvertex.at<double>(1, 2) / newvertex.at<double>(2, 2);

    newvertex.at<double>(0, 3) = newvertex.at<double>(0, 3) / newvertex.at<double>(2, 3);
    newvertex.at<double>(1, 3) = newvertex.at<double>(1, 3) / newvertex.at<double>(2, 3);

    cv::Point2d crop_p1(cropvertex_.at<double>(0, 0), cropvertex_.at<double>(1, 0));
    cv::Point2d crop_p2(cropvertex_.at<double>(0, 1), cropvertex_.at<double>(1, 1));
    cv::Point2d crop_p3(cropvertex_.at<double>(0, 2), cropvertex_.at<double>(1, 2));
    cv::Point2d crop_p4(cropvertex_.at<double>(0, 3), cropvertex_.at<double>(1, 3));
    cv::Point2d crop_cen((crop_p1.x + crop_p2.x + crop_p3.x + crop_p4.x)/4, (crop_p1.y + crop_p2.y + crop_p3.y + crop_p4.y)/4);

    if(isInside(cropvertex_, newvertex)){
        return cv::Mat::eye(3, 3, CV_64F);
    }else{
        cv::Mat comp_clone = comp.clone();
        bool all_inside = false;
        int loop = 0;
        cv::Mat move = cv::Mat::eye(3, 3, CV_64F);
        while (!all_inside && loop<5){
            cv::Mat move_t = cv::Mat::eye(3, 3, CV_64F);
            cv::Point2d p1(newvertex.at<double>(0, 0), newvertex.at<double>(1, 0));
            cv::Point2d p2(newvertex.at<double>(0, 1), newvertex.at<double>(1, 1));
            cv::Point2d p3(newvertex.at<double>(0, 2), newvertex.at<double>(1, 2));
            cv::Point2d p4(newvertex.at<double>(0, 3), newvertex.at<double>(1, 3));
            cv::Point2d comp_cen((p1.x + p2.x + p3.x + p4.x)/4, (p1.y + p2.y + p3.y + p4.y)/4);
            move_t.at<double>(0, 2) = -((comp_cen - crop_cen).x);
            move_t.at<double>(1, 2) = -((comp_cen - crop_cen).y);
            move = move * move_t;
            cv::Mat test = comp_clone * move * vertex_;
            test.at<double>(0, 0) = test.at<double>(0, 0) / test.at<double>(2, 0);
            test.at<double>(1, 0) = test.at<double>(1, 0) / test.at<double>(2, 0);

            test.at<double>(0, 1) = test.at<double>(0, 1) / test.at<double>(2, 1);
            test.at<double>(1, 1) = test.at<double>(1, 1) / test.at<double>(2, 1);

            test.at<double>(0, 2) = test.at<double>(0, 2) / test.at<double>(2, 2);
            test.at<double>(1, 2) = test.at<double>(1, 2) / test.at<double>(2, 2);

            test.at<double>(0, 3) = test.at<double>(0, 3) / test.at<double>(2, 3);
            test.at<double>(1, 3) = test.at<double>(1, 3) / test.at<double>(2, 3);
            all_inside = isInside(cropvertex_, test);
            loop++;
        }

        return move;
    }
}

cv::Mat AutoFilter::processGlobalTrans(const cv::Mat& comp, int target, int offset) {
    std::vector<cv::Mat> trans_temp;
    for(int i = 0; i < global_trans_.size(); i++){
        trans_temp.push_back(global_trans_[i]);
    }
    cv::Mat trans = calGlobalTrans(comp, crop_rate_, cv::Size(1080, 1920));
//    return trans;
    trans_temp.push_back(trans);
    int tar = trans_temp.size() - 1;
    int global_trans_size = global_trans_.size();
//    for(int i = 1; i <= global_trans_size; i++ ){
//        cv::Mat tt = cv::Mat::zeros(3, 3, CV_64F);
//        double sum_weight = 0;
//        for(int j = i, k = 0; j < window_.size() - i; j++, k++){
//            tt += weight_vec_[k+offset+i] * window_[j];
//            sum_weight += weight_vec_[k+offset+i];
//        }
//        tt /= sum_weight;
//        trans = calGlobalTrans(tt, crop_rate_, cv::Size(1080, 1920));
//        trans_temp.push_back(trans);
//    }
    std::vector<double> wei;
    wei.resize(trans_temp.size());
    for(int i = 0; i < trans_temp.size(); i++){
        int k = i - tar;
        wei[i] = filterWeight(k, 5);
    }
    cv::Mat glo_t = cv::Mat::zeros(3, 3, CV_64F);
    double sum_weight = 0;
    for(int i = 0; i < trans_temp.size(); i++){
        glo_t += wei[i] * trans_temp[i];
        sum_weight += wei[i];
    }
    glo_t /= sum_weight;
    std::cout << "sum_weitht:" << sum_weight << std::endl;
    if(global_trans_.size() == 24){
        global_trans_.pop_front();
    }
    global_trans_.push_back(glo_t);
    return glo_t;
}

cv::Mat AutoFilter::processTrans(const cv::Mat &comp, const cv::Size &size) {
    cv::Point2d pb[4];
    cv::Mat stableVec = comp.clone();
    cv::Mat s_inv = stableVec.inv();
    cv::Mat box = s_inv * cropvertex_;
    pb[0].x=box.at<double>(0,0)/box.at<double>(2,0);
    pb[0].y=box.at<double>(1,0)/box.at<double>(2,0);

    pb[1].x=box.at<double>(0,1)/box.at<double>(2,1);
    pb[1].y=box.at<double>(1,1)/box.at<double>(2,1);

    pb[2].x=box.at<double>(0,2)/box.at<double>(2,2);
    pb[2].y=box.at<double>(1,2)/box.at<double>(2,2);

    pb[3].x=box.at<double>(0,3)/box.at<double>(2,3);
    pb[3].y=box.at<double>(1,3)/box.at<double>(2,3);
    double xmin = 0 - pb[0].x ;
    double xmax = size.width - pb[0].x ;
    double ymin = 0 - pb[0].y ;
    double ymax = size.height - pb[0].y ;

    for(int j=1; j<4; j++)
    {
        if(xmin < 0 - pb[j].x)
            xmin = 0 - pb[j].x;
        if(xmax > size.width - pb[j].x)
            xmax = size.width - pb[j].x;
        if(ymin < 0 - pb[j].y)
            ymin = 0 - pb[j].y;
        if(ymax > size.height - pb[j].y)
            ymax = size.height - pb[j].y;
    }
//    file_w << frame_count <<" " << xmin << " " << xmax << " " << ymin << " " << ymax << " ";
    cv::Mat newvertex = stableVec * vertex_;
    newvertex.at<double>(0, 0) = newvertex.at<double>(0, 0) /newvertex.at<double>(2, 0);
    newvertex.at<double>(1, 0) = newvertex.at<double>(1, 0) /newvertex.at<double>(2, 0);

    newvertex.at<double>(0, 1) = newvertex.at<double>(0, 1) /newvertex.at<double>(2, 1);
    newvertex.at<double>(1, 1) = newvertex.at<double>(1, 1) /newvertex.at<double>(2, 1);

    newvertex.at<double>(0, 2) = newvertex.at<double>(0, 2) /newvertex.at<double>(2, 2);
    newvertex.at<double>(1, 2) = newvertex.at<double>(1, 2) /newvertex.at<double>(2, 2);

    newvertex.at<double>(0, 3) = newvertex.at<double>(0, 3) /newvertex.at<double>(2, 3);
    newvertex.at<double>(1, 3) = newvertex.at<double>(1, 3) /newvertex.at<double>(2, 3);

    bool allInside=isInside(cropvertex_,newvertex);

    cv::Mat resultVec=stableVec.clone();
    if(!allInside) {
        double ratio=1.0;
        cv::Mat I=cv::Mat::eye(3, 3, CV_64F);
        while ((!allInside) && (ratio >= 0)) {
            double transdet = determinant(stableVec);
            cv::Mat transtemp = stableVec / pow(transdet, 1.0 / 3);
            resultVec = I * (1 - ratio) + transtemp * ratio;

            ratio = ratio - 0.01;
            newvertex = resultVec * vertex_;
            newvertex.at<double>(0, 0) = newvertex.at<double>(0, 0) / newvertex.at<double>(2, 0);
            newvertex.at<double>(1, 0) = newvertex.at<double>(1, 0) / newvertex.at<double>(2, 0);

            newvertex.at<double>(0, 1) = newvertex.at<double>(0, 1) / newvertex.at<double>(2, 1);
            newvertex.at<double>(1, 1) = newvertex.at<double>(1, 1) / newvertex.at<double>(2, 1);

            newvertex.at<double>(0, 2) = newvertex.at<double>(0, 2) / newvertex.at<double>(2, 2);
            newvertex.at<double>(1, 2) = newvertex.at<double>(1, 2) / newvertex.at<double>(2, 2);

            newvertex.at<double>(0, 3) = newvertex.at<double>(0, 3) / newvertex.at<double>(2, 3);
            newvertex.at<double>(1, 3) = newvertex.at<double>(1, 3) / newvertex.at<double>(2, 3);

            allInside = isInside(cropvertex_, newvertex);
        }
    }

    double x_m, y_m;
    if(xmin > xmax || ymin > ymax)
    {
//        std::cout << xmin << " " << xmax << " " << ymin << " " << ymax << std::endl;
//        std::cout<<"Size ERROR! use intr!"<<std::endl;
        return resultVec;
    }
    else {
        double x_mid = (xmin + xmax) / 10;
        double y_mid = (ymin + ymax) / 10;
//        if(count_%2 == 0){
//            x_mid = 0;
//            y_mid = 0;
//        }
        int q_posi = 0;

        if (count_ < q_size) {
            q_posi = count_;
            count_++;
        } else {
            q_posi = q_size - 1;
            count_++;
        }

        if (count_ < q_pow + 2) {
            queue_in(x_q, q_posi, x_mid);
            queue_in(y_q, q_posi, y_mid);
            x_m = x_mid;
            y_m = y_mid;

        } else {
            queue_in(x_q, q_posi, x_mid);
            queue_in(y_q, q_posi, y_mid);
            x_m = polyfit(f_num, x_q, q_posi + 1, q_pow, q_posi + 1);
//            x_m = gfilte(x_q, q_posi+1, q_posi+1);
            y_m = polyfit(f_num, y_q, q_posi + 1, q_pow, q_posi + 1);
//            y_m = gfilte(y_q, q_posi+1, q_posi+1);

        }


//        std::cout << "Move adjust: " << " x: " << x_m << ", y:" << y_m << std::endl;

        if (x_m < xmin) {
            std::cout << "adjust x is too small" << std::endl;
            x_m = xmin;
        } else if (x_m > xmax) {
            std::cout << "adjust x is too big" << std::endl;
            x_m = xmax;
        }

        if (y_m < ymin) {
            std::cout << "adjust y is too small" << std::endl;
            y_m = ymin;
        } else if (y_m > ymax) {
            std::cout << "adjust y is too big" << std::endl;
            y_m = ymax;
        }
//        file_w << x_m << " " << y_m << std::endl;

        cv::Mat comp2 = cv::Mat::eye(3, 3, CV_64F);
        comp2.at<double>(0, 2) = x_m;
        comp2.at<double>(1, 2) = y_m;

        cv::Mat news = (comp2 * s_inv).inv();
        return news;
    }
}

void AutoFilter::queue_in(double *q, int m, double x) {
    for(int i=1; i<=m; i++)
    {
        q[i-1] = q[i];
    }
    q[m]=x;
}


double AutoFilter::polyfit(double *arrX, double *arrY, int num, int n, double x) {
    int size = num;
    int x_num = n + 1;
    //构造矩阵U和Y
    cv::Mat mat_u(size, x_num, CV_64F);
    cv::Mat mat_y(size, 1, CV_64F);

    for (int i = 0; i < mat_u.rows; ++i)
        for (int j = 0; j < mat_u.cols; ++j)
        {
            mat_u.at<double>(i, j) = pow(arrX[i], j);
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

    double y = 0;
    for (int j = 0; j < n + 1; ++j)
    {
        y += mat_k.at<double>(j, 0)*pow(x,j);
    }
    return y;
}