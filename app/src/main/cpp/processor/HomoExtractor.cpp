//
// Created by ShiJJ on 2020/6/1.
//

#include "HomoExtractor.h"
#define LOG_TAG    "c_ThreadCompensation"
#define LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...)  __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
double HomoExtractor::point_distance(cv::Point2f p1, cv::Point2f p2) {
    cv::Point2f d = p1 - p2;
    double d_mu;
    d_mu = sqrt(d.x * d.x + d.y * d.y);
    return d_mu;
}
bool HomoExtractor::outOfImg(const cv::Point2f &point, const cv::Size &size) {
    return (point.x <= 0 || point.y <= 0 || point.x >= size.width - 1 || point.y >= size.height - 1 );
}

int HomoExtractor::in_area(cv::Point2f p1, int w, int h) {
    int res1,res2;
    if(p1.x<w)
    {
        res1 = 0;
    }
    else if(p1.x>=w && p1.x<2*w)
    {
        res1 = 1;
    }
    else if(p1.x>=2*w && p1.x<3*w)
    {
        res1 = 2;
    }
    else
    {
        res1 = 3;
    }

    if(p1.y<h)
    {
        res2 = 0;
    }
    else if(p1.y>=h && p1.y<2*h)
    {
        res2 = 1;
    }
    else if(p1.y>=2*h && p1.y<3*h)
    {
        res2 = 2;
    }
    else
    {
        res2 = 3;
    }

    return res1 * 4 + res2;
}

bool HomoExtractor::judge_area() {
    block_index_.clear();
    bool redetect = false;
    int num[16] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    for (int i = 0; i < 16; i++) {
        if (point_num[i] < num[i]) {
            redetect = true;
            block_index_.push_back(i);
        }
    }

    return redetect;
}

bool HomoExtractor::judge_recal_simple(cv::Mat img1, std::vector<char> ifselect) {
    int half_w=img1.cols/2 , half_h=img1.rows/2;
    double inli[4]={0,0,0,0}, p_num[4]={0,0,0,0};
    for(int i=0; i<curFeaturesTmp.size(); i++)
    {
        if(i>=ifselect.size())
        {
            break;
        }
        int i1,i2;
        if(curFeaturesTmp[i].x<half_w)
        {
            i1=0;
        }
        else{
            i1=1;
        }
        if(curFeaturesTmp[i].y<half_h)
        {
            i2=2;
        }
        else{
            i2=3;
        }
        p_num[i1]++;
        p_num[i2]++;

        if(ifselect[i]==1)
        {
            inli[i1]++;
            inli[i2]++;
        }
    }

    double rate[4];
    for(int i=0;i<4;i++)
    {
        rate[i]=double(inli[i])/double(p_num[i]);
    }
    bool re=false;
    double limit=0.25;
    for(int i=0;i<4;i++)
    {
        if((rate[i]<limit || inli[i]<2) && p_num[i]>1)
        {
            re=true;
        }
    }

    return re;
}

void HomoExtractor::detectFeature() {
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

std::vector<cv::Point2f> HomoExtractor::detectCertainFeature(const std::vector<int>& block_index) {
    std::vector<cv::Point2f> ret_feature;
    double quality_level = 0.1;
    double min_distance = 8;
    int max_corners = 8;
    std::vector<cv::Point2f> lastFeatures_a;
    int half_w=lastGray.cols/4 , half_h=lastGray.rows/4;
    cv::Mat lastGray_a;
    for(auto i : block_index){
        int x_i = i % 4;
        int y_i = i / 4;
        cv::Rect rect(y_i*half_w, x_i*half_h, half_w, half_h);
        lastGray_a = lastGray(rect);
        goodFeaturesToTrack(lastGray_a, lastFeatures_a,  max_corners, quality_level, min_distance);
        cv::Point2f temp(y_i*half_w, x_i*half_h);
        for(int j=0; j<lastFeatures_a.size(); j++)
        {
            cv::Point2f pt=lastFeatures_a[j]+temp;//转化成全局坐标
            ret_feature.push_back(pt);
        }
    }
    return ret_feature;
}

void HomoExtractor::trackCertainFeature( std::vector<cv::Point2f> &last_add_feature) {
    double rate = 1.4;
    std::vector<cv::Point2f> cur_add_feature;
    std::vector<uchar> add_status;
    std::vector<float> err;
    calcOpticalFlowPyrLK( lastGray , curGray , last_add_feature , cur_add_feature , add_status , err);
    int max = last_add_feature.size() < cur_add_feature.size() ? last_add_feature.size() : cur_add_feature.size();
    double dis_sum=0;
    for(int i=0;i<max;i++)
    {
        dis_sum += point_distance(last_add_feature[i],cur_add_feature[i]);
    }
    double dis_avg=0;
    dis_avg=dis_sum/max;
    max = max < add_status.size() ? max : add_status.size();
    for(int i=0;i<max;i++)
    {
        if(point_distance(last_add_feature[i],cur_add_feature[i]) > dis_avg * rate)
        {
            add_status[i] = 0;
        }
    }

    cv::Mat m_Fundamental;
    std::vector<uchar> m_RANSACStatus;
    cv::Mat p1(last_add_feature);
    cv::Mat p2(cur_add_feature);
    double outliner=0;
    m_Fundamental = findFundamentalMat(p1, p2, m_RANSACStatus, cv::FM_RANSAC, 3, 0.99);
    for (int j = 0 ; j < add_status.size() ; j++ )
    {
        if (m_RANSACStatus.size() > j && m_RANSACStatus[j] == 0) // 状态为0表示野点(误匹配)
        {
            add_status[j] = 0;
        }
        if(add_status[j]==0)
        {
            outliner++;
        }
    }

    int new_point_num[16] = {0};
    for(auto pt : cur_add_feature){
        int a = in_area(pt, lastGray.cols/4, lastGray.rows/4);
        new_point_num[a]++;
    }
    LOGI("rerererere:new[%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
         new_point_num[0], new_point_num[1],  new_point_num[2], new_point_num[3],
         new_point_num[4],  new_point_num[5],  new_point_num[6], new_point_num[7],
         new_point_num[8],  new_point_num[9],  new_point_num[10], new_point_num[11],
         new_point_num[12], new_point_num[13], new_point_num[14], new_point_num[15]);


    statussize += max;
    int idx = 0;
    for (auto itC = cur_add_feature.begin(), itP = last_add_feature.begin(); itC != cur_add_feature.end(); itC ++, itP ++, idx ++) {

        if (status[idx] == 0 || err[idx] > 20 || outOfImg(*itC, cv::Size(lastGray.cols, lastGray.rows))) {

        } else {
            cv::Point2f cfp=*itC * ThreadContext::DOWNSAMPLE_SCALE;
            cv::Point2f lfp=*itP * ThreadContext::DOWNSAMPLE_SCALE;
            curFeaturesTmp.push_back(cfp);
            lastFeaturesTmp.push_back(lfp);
        }
    }
}

void HomoExtractor::trackFeature(const cv::Mat &img1, const cv::Mat &img2) {
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

    statussize=max;
    int half_w=img1.cols/4 , half_h=img1.rows/4;
    cv::Point2f center(img1.cols / 2, img1.rows / 2);
    int idx = 0;
    curFeaturesTmp.clear();
    lastFeaturesTmp.clear();
    for(int j=0; j<16; j++)
    {
        point_num[j] = 0;
    }

    //LOGI("step4_6");
    for (auto itC = curFeatures.begin(), itP = lastFeatures.begin(); itC != curFeatures.end(); itC ++, itP ++, idx ++) {

        if (status[idx] == 0 || err[idx] > 20 || outOfImg(*itC, cv::Size(lastGray.cols, lastGray.rows))) {
            status_choose[idx]=0;
        } else {
            cv::Point2f cfp=*itC * ThreadContext::DOWNSAMPLE_SCALE;
            cv::Point2f lfp=*itP * ThreadContext::DOWNSAMPLE_SCALE;
            curFeaturesTmp.push_back(cfp);
            lastFeaturesTmp.push_back(lfp);
            int a = in_area(cfp, half_w, half_h);
            point_num[a]++;
        }
    }
}

void HomoExtractor::calcul_Homo(std::vector<char> &ifselect, int niter, int type) {
    H = cv::Mat();
    ifselect.clear();
    if(lastFeaturesTmp.size() < 2)
    {
        H = cv::Mat::eye(3, 3, CV_64F);
    }
    else if(niter<0)
    {
        H = findHomography(lastFeaturesTmp, curFeaturesTmp, 0);
    }
    else
    {
        if (!lastFeaturesTmp.empty() && !curFeaturesTmp.empty() && lastFeaturesTmp.size() > 3) {

            if(type==0)
            {
                H = findHomography(lastFeaturesTmp, curFeaturesTmp, cv::LMEDS, 3, ifselect, niter, 0.995);
            }
            else
            {
                H = findHomography(lastFeaturesTmp, curFeaturesTmp, cv::RHO, 4, ifselect, niter, 0.995);
            }

        }
        else if(!lastFeaturesTmp.empty() && !curFeaturesTmp.empty() && lastFeaturesTmp.size() > 1)
        {
            if(lastFeaturesTmp.size()==3)
            {
                cv::Mat AF=getAffineTransform(lastFeaturesTmp,curFeaturesTmp);
                H=cv::Mat::zeros(3,3,AF.type());

                H.at<double>(0,0)=AF.at<double>(0,0);
                H.at<double>(0,1)=AF.at<double>(0,1);
                H.at<double>(0,2)=AF.at<double>(0,2);
                H.at<double>(1,0)=AF.at<double>(1,0);
                H.at<double>(1,1)=AF.at<double>(1,1);
                H.at<double>(1,2)=AF.at<double>(1,2);
                H.at<double>(2,0)=0;
                H.at<double>(2,1)=0;
                H.at<double>(2,2)=1;

            }
            else if(lastFeaturesTmp.size()==2)
            {
                std::vector<cv::Point2f> curFeat_2, lastFeat_2;
                curFeat_2.push_back(curFeaturesTmp[0]);
                lastFeat_2.push_back(lastFeaturesTmp[0]);
                curFeat_2.push_back(curFeaturesTmp[1]);
                lastFeat_2.push_back(lastFeaturesTmp[1]);

                cv::Point2f cp(curFeaturesTmp[1].x-(curFeaturesTmp[1].y-curFeaturesTmp[0].y),curFeaturesTmp[1].y-(curFeaturesTmp[1].x-curFeaturesTmp[0].x));
                cv::Point2f lp(lastFeaturesTmp[1].x-(lastFeaturesTmp[1].y-lastFeaturesTmp[0].y),lastFeaturesTmp[1].y-(lastFeaturesTmp[1].x-lastFeaturesTmp[0].x));
                curFeat_2.push_back(cp);
                lastFeat_2.push_back(lp);

                cv::Mat AF = getAffineTransform(lastFeat_2, curFeat_2);
                H=cv::Mat::zeros(3,3,AF.type());

                H.at<double>(0, 0) = AF.at<double>(0, 0);
                H.at<double>(0, 1) = AF.at<double>(0, 1);
                H.at<double>(0, 2) = AF.at<double>(0, 2);
                H.at<double>(1, 0) = AF.at<double>(1, 0);
                H.at<double>(1, 1) = AF.at<double>(1, 1);
                H.at<double>(1, 2) = AF.at<double>(1, 2);
                H.at<double>(2, 0) = 0;
                H.at<double>(2, 1) = 0;
                H.at<double>(2, 2) = 1;
            }
        }
    }
}

cv::Point2f HomoExtractor::goround(cv::Point2f p1, cv::Point2f p0, double degree) {
    double ro = PI / 180 * degree;
    double xr=(p1.x-p0.x)*cos(ro) - (p1.y-p0.y)*sin(ro) + p0.x;
    double yr=(p1.x-p0.x)*sin(ro) + (p1.y-p0.y)*cos(ro) + p0.y;
    cv::Point2f pr(xr,yr);
    return pr;
}

cv::Point2f HomoExtractor::goscale(cv::Point2f p1, cv::Point2f p0, double scale) {
    cv::Point2f pr=p0+(p1-p0)*scale;
    return pr;
}

double HomoExtractor::vec_cos(cv::Point2f s, cv::Point2f e1, cv::Point2f e2) {
    cv::Point2f v1=e1-s, v2=e2-s;
    cv::Point2f o(0,0);
    double l1=point_distance(o,v1), l2=point_distance(o,v2);
    double y = (v1.x * v2.x + v1.y * v2.y)/(l1 * l2);

    return y;
}

double HomoExtractor::calcul_H_error(int c_h, int c_w) {
    int height=100;
    int width=100;
    double error_rate=1.1;
    bool aff_adj=false;

    cv::Mat  vertex=(cv::Mat_<double>(3, 4)<<c_w-width, c_w-width, c_w+width, c_w+width, c_h-height, c_h+height, c_h+height, c_h-height, 1.0, 1.0, 1.0, 1.0);

    cv::Point2f cp1, cp2, cp3, cp4;
    std::vector<cv::Point2f> l, c;

    cv::Mat newvertex = H * vertex;
    cp1.x = newvertex.at<double>(0, 0) / newvertex.at<double>(2, 0);
    cp1.y = newvertex.at<double>(1, 0) / newvertex.at<double>(2, 0);

    cp2.x = newvertex.at<double>(0, 1) / newvertex.at<double>(2, 1);
    cp2.y = newvertex.at<double>(1, 1) / newvertex.at<double>(2, 1);

    cp3.x = newvertex.at<double>(0, 2) / newvertex.at<double>(2, 2);
    cp3.y = newvertex.at<double>(1, 2) / newvertex.at<double>(2, 2);

    cp4.x = newvertex.at<double>(0, 3) / newvertex.at<double>(2, 3);
    cp4.y = newvertex.at<double>(1, 3) / newvertex.at<double>(2, 3);

    double c_x=(cp1.x + cp2.x + cp3.x + cp4.x)/4;
    double c_y=(cp1.y + cp2.y + cp3.y + cp4.y)/4;
    cv::Point2f pc(c_x,c_y);

    cv::Point2f lp1(c_x-width, c_y-height),lp2(c_x-width, c_y+height),lp3(c_x+width, c_y+height),lp4(c_x+width, c_y-height);
    double mindis=9999;
    double mind=0;

    for(double d=-10; d<=10; d++)
    {
        cv::Point2f rp1=goround(lp1,pc,d);
        cv::Point2f rp2=goround(lp2,pc,d);
        cv::Point2f rp3=goround(lp3,pc,d);
        cv::Point2f rp4=goround(lp4,pc,d);

        double l1 = point_distance(cp1,rp1);
        double l2 = point_distance(cp2,rp2);
        double l3 = point_distance(cp3,rp3);
        double l4 = point_distance(cp4,rp4);

        if(l1+l2+l3+l4<mindis)
        {
            mindis=l1+l2+l3+l4;
            mind=d;
        }
    }

    double mindis2=9999;
    double mins=0;
    cv::Point2f rp1=goround(lp1,pc,mind);
    cv::Point2f rp2=goround(lp2,pc,mind);
    cv::Point2f rp3=goround(lp3,pc,mind);
    cv::Point2f rp4=goround(lp4,pc,mind);
    for(double s=0.9; s<=1.1; s=s+0.01)
    {
        cv::Point2f sp1=goscale(rp1,pc,s);
        cv::Point2f sp2=goscale(rp2,pc,s);
        cv::Point2f sp3=goscale(rp3,pc,s);
        cv::Point2f sp4=goscale(rp4,pc,s);

        double l1 = point_distance(cp1,sp1);
        double l2 = point_distance(cp2,sp2);
        double l3 = point_distance(cp3,sp3);
        double l4 = point_distance(cp4,sp4);

        if(l1+l2+l3+l4<mindis2)
        {
            mindis2=l1+l2+l3+l4;
            mins=s;
        }
    }

    if(true)
    {
        double d1=vec_cos(cp1,cp2,cp4);
        double d2=vec_cos(cp2,cp1,cp3);
        double d3=vec_cos(cp3,cp2,cp4);
        double d4=vec_cos(cp4,cp1,cp3);

        stable_move=true;
        stable_move2=false;
        if((d1<0 && d2<0) || (d2<0 && d3<0) || (d3<0 && d4<0) || (d4<0 && d1<0))
        {
            stable_move=false;
        }
        double limit2=0.015;
        if((d1>limit2 && d3>limit2) || (d2>limit2 && d4>limit2))
        {
            stable_move2=true;
        }
    }

    if(true)
    {
        cp1=goround(lp1,pc,mind);
        cp2=goround(lp2,pc,mind);
        cp3=goround(lp3,pc,mind);
        cp4=goround(lp4,pc,mind);

        cp1=goscale(cp1,pc,mins);
        cp2=goscale(cp2,pc,mins);
        cp3=goscale(cp3,pc,mins);
        cp4=goscale(cp4,pc,mins);

        lp1=cv::Point2f(c_w-width, c_h-height);
        lp2=cv::Point2f(c_w-width, c_h+height);
        lp3=cv::Point2f(c_w+width, c_h+height);
        lp4=cv::Point2f(c_w+width, c_h-height);

        std::vector<cv::Point2f> lpt,cpt;
        lpt.push_back(lp1);
        lpt.push_back(lp2);
        lpt.push_back(lp3);
        lpt.push_back(lp4);

        cpt.push_back(cp1);
        cpt.push_back(cp2);
        cpt.push_back(cp3);
        cpt.push_back(cp4);

        second_H = findHomography(lpt, cpt, 0);
    }

    return mindis;
}

void HomoExtractor::stab_feature_25(cv::Mat img1, cv::Mat img2) {
    int num=(curFeaturesTmp.size()<lastFeaturesTmp.size()?curFeaturesTmp.size():lastFeaturesTmp.size());
    cv::Point2f sum_d(0,0);
    for(int i=0;i<num;i++)
    {
        cv::Point2f d = curFeaturesTmp[i] - lastFeaturesTmp[i];
        sum_d += d;
    }
    cv::Point2f avg_d = sum_d/num;

    int half_w=img1.cols/3 , half_h=img1.rows/3;
    std::vector<cv::Point2f> last_add, cur_add;
    last_add.push_back(cv::Point2f(0,0));
    last_add.push_back(cv::Point2f(0,half_h));
    last_add.push_back(cv::Point2f(0,2 * half_h));
    last_add.push_back(cv::Point2f(0,3 * half_h));
    last_add.push_back(cv::Point2f(half_w,0));
    last_add.push_back(cv::Point2f(half_w,half_h));
    last_add.push_back(cv::Point2f(half_w,2 * half_h));
    last_add.push_back(cv::Point2f(half_w,3 * half_h));
    last_add.push_back(cv::Point2f(2 * half_w,0));
    last_add.push_back(cv::Point2f(2 * half_w,half_h));
    last_add.push_back(cv::Point2f(2 * half_w,2 * half_h));
    last_add.push_back(cv::Point2f(2 * half_w,3 * half_h));
    last_add.push_back(cv::Point2f(3 * half_w,0));
    last_add.push_back(cv::Point2f(3 * half_w,half_h));
    last_add.push_back(cv::Point2f(3 * half_w,2 * half_h));
    last_add.push_back(cv::Point2f(3 * half_w,3 * half_h));

    last_add.push_back(cv::Point2f(half_w/2,half_h/2));
    last_add.push_back(cv::Point2f(half_w/2,half_h*3/2));
    last_add.push_back(cv::Point2f(half_w/2,half_h*5/2));
    last_add.push_back(cv::Point2f(half_w*3/2,half_h/2));
    last_add.push_back(cv::Point2f(half_w*3/2,half_h*3/2));
    last_add.push_back(cv::Point2f(half_w*3/2,half_h*5/2));
    last_add.push_back(cv::Point2f(half_w*5/2,half_h/2));
    last_add.push_back(cv::Point2f(half_w*5/2,half_h*3/2));
    last_add.push_back(cv::Point2f(half_w*5/2,half_h*5/2));

    for(int i=0;i<last_add.size();i++)
    {
        cur_add.push_back(last_add[i]+avg_d);
    }

    for(int i=0;i<last_add.size();i++)
    {
        lastFeaturesTmp.push_back(last_add[i]);
        curFeaturesTmp.push_back(cur_add[i]);
    }

    for(int i=0;i<last_add.size();i++)
    {
        lastFeaturesTmp.push_back(last_add[i]);
        curFeaturesTmp.push_back(cur_add[i]);
    }
    for(int i=0;i<last_add.size();i++)
    {
        lastFeaturesTmp.push_back(last_add[i]);
        curFeaturesTmp.push_back(cur_add[i]);
    }
}

void HomoExtractor::stab_feature_25_H(cv::Mat img1, cv::Mat img2) {
    int half_w=img1.cols/3 , half_h=img1.rows/3;
    std::vector<cv::Point2f> last_add, cur_add;
    last_add.push_back(cv::Point2f(0,0));
    last_add.push_back(cv::Point2f(0,half_h));
    last_add.push_back(cv::Point2f(0,2 * half_h));
    last_add.push_back(cv::Point2f(0,3 * half_h));
    last_add.push_back(cv::Point2f(half_w,0));
    last_add.push_back(cv::Point2f(half_w,half_h));
    last_add.push_back(cv::Point2f(half_w,2 * half_h));
    last_add.push_back(cv::Point2f(half_w,3 * half_h));
    last_add.push_back(cv::Point2f(2 * half_w,0));
    last_add.push_back(cv::Point2f(2 * half_w,half_h));
    last_add.push_back(cv::Point2f(2 * half_w,2 * half_h));
    last_add.push_back(cv::Point2f(2 * half_w,3 * half_h));
    last_add.push_back(cv::Point2f(3 * half_w,0));
    last_add.push_back(cv::Point2f(3 * half_w,half_h));
    last_add.push_back(cv::Point2f(3 * half_w,2 * half_h));
    last_add.push_back(cv::Point2f(3 * half_w,3 * half_h));

    last_add.push_back(cv::Point2f(half_w/2,half_h/2));
    last_add.push_back(cv::Point2f(half_w/2,half_h*3/2));
    last_add.push_back(cv::Point2f(half_w/2,half_h*5/2));
    last_add.push_back(cv::Point2f(half_w*3/2,half_h/2));
    last_add.push_back(cv::Point2f(half_w*3/2,half_h*3/2));
    last_add.push_back(cv::Point2f(half_w*3/2,half_h*5/2));
    last_add.push_back(cv::Point2f(half_w*5/2,half_h/2));
    last_add.push_back(cv::Point2f(half_w*5/2,half_h*3/2));
    last_add.push_back(cv::Point2f(half_w*5/2,half_h*5/2));

    for(int i=0;i<last_add.size();i++)
    {
        double cx=second_H.at<double>(0,0)*last_add[i].x + second_H.at<double>(0,1)*last_add[i].y + second_H.at<double>(0,2);
        double cy=second_H.at<double>(1,0)*last_add[i].x + second_H.at<double>(1,1)*last_add[i].y + second_H.at<double>(1,2);
        double cw=second_H.at<double>(2,0)*last_add[i].x + second_H.at<double>(2,1)*last_add[i].y + second_H.at<double>(2,2);
        cur_add.push_back(cv::Point2f(cx/cw, cy/cw));
    }

    for(int i=0;i<last_add.size();i++)
    {
        lastFeaturesTmp.push_back(last_add[i]);
        curFeaturesTmp.push_back(cur_add[i]);
    }

    for(int i=0;i<last_add.size();i++)
    {
        lastFeaturesTmp.push_back(last_add[i]);
        curFeaturesTmp.push_back(cur_add[i]);
    }
    for(int i=0;i<last_add.size();i++)
    {
        lastFeaturesTmp.push_back(last_add[i]);
        curFeaturesTmp.push_back(cur_add[i]);
    }
}

cv::Mat HomoExtractor::extractHomo(cv::Mat &img1, cv::Mat &img2) {
//    cv::Mat img2_r;
    curGray = img2.rowRange(0,img2.rows * 2 / 3);
    cv::resize(curGray, curGray, cv::Size(curGray.cols / ThreadContext::DOWNSAMPLE_SCALE, curGray.rows / ThreadContext::DOWNSAMPLE_SCALE));
//    cv::cvtColor(img2_r, curGray , cv::COLOR_BGR2GRAY );

    if(ex_index_ == 0){
//        cv::Mat img1_r;
        lastGray = img1.rowRange(0,img1.rows * 2 / 3);
        cv::resize(lastGray, lastGray, cv::Size(lastGray.cols / ThreadContext::DOWNSAMPLE_SCALE, lastGray.rows / ThreadContext::DOWNSAMPLE_SCALE));
//        cv::cvtColor(img1_r, lastGray , cv::COLOR_BGR2GRAY );
        detectFeature();
    }
    trackFeature(img1.rowRange(0,img2.rows * 2 / 3), img2.rowRange(0,img2.rows * 2 / 3));

    bool re = judge_area();

    if(re){
//        detectFeature();
//        trackFeature(img1.rowRange(0,img2.rows * 2 / 3),img2.rowRange(0,img2.rows * 2 / 3));
        auto last_add_feature = detectCertainFeature(block_index_);
        if(last_add_feature.size() != 0){
            trackCertainFeature(last_add_feature);
        }


//        LOGI("rerererere:[%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
//                point_num[0], point_num[1],  point_num[2], point_num[3],
//             point_num[4],  point_num[5],  point_num[6], point_num[7],
//             point_num[8],  point_num[9],  point_num[10], point_num[11],
//             point_num[12], point_num[13], point_num[14], point_num[15]);
//        LOGI("rerererere:%d", block_index_[0]);

    }
    std::vector<char> ifselect;
    calcul_Homo(ifselect,2000,0);


    int height=img1.rows;
    int width=img1.cols;
    bool isValidH = !H.empty();
    int half_w=img1.cols/4 , half_h=img1.rows/4;
    int tmp_count[16]={0};
    double ifinliner = 0;

    for(int i=0; i<ifselect.size(); i++)
    {
        if(ifselect[i] == 1)
        {
            ifinliner++;
        }
    }

    double ifinrate1=ifinliner/statussize;
    double ifinrate2=ifinliner/ifselect.size();
    bool re2 = false;
    re2= judge_recal_simple(img1, ifselect);
    if(re2){
        calcul_Homo(ifselect,2000,1);
    }
    H_ori = H.clone();
    double h_err = calcul_H_error(height/2, width/2);
    LOGI("h_err:%f", h_err);

    bool jiaozheng=true;
    if(ifinrate2>0.95)
    {
        jiaozheng=false;
    }

    if(ex_index_ < ThreadContext::SEGSIZE/2)
    {
        sumup_H_err1 += h_err;
    }
    else{
        sumup_H_err2 += h_err;
    }

    if(h_err > 5 && jiaozheng)
    {
        addp_frame++;
        if(h_err>13)
            stab_feature_25(img1,img2);
        else
            stab_feature_25_H(img1,img2);

        calcul_Homo(ifselect,2000,0);
        LOGI("redo detect and track");

    }

    if(draw_information){
        int tsize=curFeaturesTmp.size()>lastFeaturesTmp.size() ? lastFeaturesTmp.size() : curFeaturesTmp.size();
        for(int i = 0; i < tsize; i++){
            cv::Point2f p1 = lastFeaturesTmp[i];
            cv::Point2f p2 = curFeaturesTmp[i];
            cv::circle(img2,p1,10,cv::Scalar(255,255,0),2);
            cv::circle(img2,p2,10,cv::Scalar(255,0,0),2);
            cv::line(img2,p1,p2,cv::Scalar(0,0,255),8);
        }
    }
    threads::ThreadContext::feature_by_r_.push(curFeaturesTmp);
    lastFeatures.clear();
    lastFeatures.assign(curFeatures.begin(), curFeatures.end());
    curGray.copyTo(lastGray);
    ex_index_++;
    if(ex_index_ == ThreadContext::SEGSIZE){
        ex_index_ = 0;
    }



    return H;


}