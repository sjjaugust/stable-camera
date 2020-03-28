//
// Created by 张哲华 on 19/09/2017.
//

#include "ThreadCompensation.h"
#include <android/log.h>
#include "TimeRecorder.h"
using namespace cv;
using namespace std;
using namespace threads;

ThreadCompensation::~ThreadCompensation() {
    worker_thread_.join();//让其他线程等待知道该线程运行完成
}

void ThreadCompensation::start() {
    worker_thread_ = thread(&ThreadCompensation::worker, this);//生成新线程
}

void ThreadCompensation::worker()
{
    pthread_setname_np(pthread_self(), "CompensationThread"); // set the name (pthread_self() returns the pthread_t of the current thread
    while(true)
    {
        ThreadContext::mc_semaphore->Wait();//取已经完成特征点轨迹构造的资源，若无，线程等待
//        __android_log_print(ANDROID_LOG_DEBUG, "NThreadMC", "before");
        if( ThreadContext::motionCompList[0].y < ThreadContext::SEGSIZE )
        {
//            ThreadContext::out_semaphore->Signal();
            ThreadContext::rs_semaphore->Signal();
            break;
        }
        frameCompensate();


        ThreadContext::motionCompList.erase( ThreadContext::motionCompList.begin() );
//        __android_log_print(ANDROID_LOG_DEBUG, "NThreadMC", "after");
//       ThreadContext::out_semaphore->Signal();//唤醒显示和保存线程
        ThreadContext::rs_semaphore->Signal();
    }
}

void ThreadCompensation::computeAffine( vector<Point2f> &avgFeatPos , vector<Mat> &affineMatrix )
{
    list<list<vector<Point2f>>>::iterator Iter = ThreadContext::trj.begin();//指向第一个list<vector<point2f>>
    if( (*Iter).size() < 3 )//计算仿射矩阵至少需要三个特征点对
        return;

    int trjNum = (int) (*Iter).size();//特征点个数
    int trjLength = (int) (*(*Iter).begin()).size();//每个vector中的元素个数,即段内帧数
    
    for( int i = 0 ; i < trjLength ; i++ )
    {
        Point2f sum(0,0);
        list<vector<Point2f>>::iterator Iter_2;//指向第二层链
        //特征点坐标之和
        for( Iter_2 = (*Iter).begin() ; Iter_2 != (*Iter).end() ; Iter_2++ )
        {
            sum += (*Iter_2)[i];//讲每帧坐标位置加和
        }
        Point2f centerPoint;
        centerPoint.x = sum.x / trjNum;//除以特征点个数
        centerPoint.y = sum.y / trjNum;
        avgFeatPos.push_back(centerPoint);//存储每一帧的中心点
    }


    //轨迹坐标中心化，转置
    vector<vector<Point2f>> normalTrj_T;
    for( int i = 0 ; i < trjLength ; i++ )
    {
        vector<Point2f> tmp;
        list<vector<Point2f>>::iterator Iter_2;
        for( Iter_2 = (*Iter).begin() ; Iter_2 != (*Iter).end() ; Iter_2++ )
        {
            (*Iter_2)[i] -= avgFeatPos[i];
            tmp.push_back( (*Iter_2)[i] );
        }//减去中心点
        normalTrj_T.push_back( tmp );
    }

    Mat affine;
    Mat conMat = Mat::zeros(1, 3, CV_64F);
    conMat.at<double>(0, 2) = 1;
    Mat tmpEye = Mat::eye(3, 3, CV_64F);
    Mat I = Mat::eye(2, 3, CV_64F);
    for( int i = 1 ; i < avgFeatPos.size() - 1 ; i++ )
    {
        //计算前一帧和后一帧的仿射矩阵，2行3列
        affine = estimateRigidTransform( normalTrj_T[i] , normalTrj_T[i - 1] , false );
        if (affine.empty()) {
            affine = I;
        }
        vconcat(affine, conMat, affine);//4行3列
        tmpEye = tmpEye * affine;
        affineMatrix.push_back(tmpEye.rowRange(0, 2).clone());
    }
    affine = estimateRigidTransform( normalTrj_T[avgFeatPos.size()-1] , normalTrj_T[avgFeatPos.size()-2] , false );
    if (affine.empty()) {
        affine = I;
    }
    vconcat(affine, conMat, affine);
    tmpEye = tmpEye * affine;
    Mat affineLast = tmpEye.inv();

    affineMatrix.push_back( affineLast.rowRange(0, 2).clone());
}

void ThreadCompensation::frameCompensate()
{
    int start = ThreadContext::motionCompList[0].x;
    int length = ThreadContext::motionCompList[0].y;

    /*稳定视频帧*/
    vector<Point2f> avgFeatPos;//存储段内每帧的几何中心坐标
    vector<Mat> affineMatrix;//存储帧与帧之间的仿射变换

    /*为旋转插值准备数据，即计算仿射矩阵，计算本段非关键帧到前关键帧的仿射矩阵与前关键帧到后关键帧的仿射矩阵*/
    computeAffine( avgFeatPos , affineMatrix );

    vector<double> thetaVec;
    for( int i = 0 ; i < affineMatrix.size() ; i++ )//进行SVD分解，得到旋转角度
    {
        if( !affineMatrix.empty() )
        {
            SVD thissvd;
            Mat affine , W , U , VT;
            affine = affineMatrix[i].colRange(0,2);
            thissvd.compute(affine,W,U,VT,SVD::FULL_UV);//计算旋转矩阵
            affine = U*VT;
            double theta = asinf((float) affine.at<double>(0, 1));
            thetaVec.push_back(theta);
        }
    }

    bool isStable = false;
    if( shakeDetect )//如果进行抖动检测，计算平移振幅和旋转振幅
    {
        if( avgFeatPos.size() > 0 )
        {
            /*计算平均平移振幅*/
            float cosinlim = 0.996;
            vector<float> shift_SD;
            for( int i = 1 ; i < avgFeatPos.size() - 1 ; i++ )
            {
                Point2f d1 , d2;
                d1 = avgFeatPos[i] - avgFeatPos[i-1];
                d2 = avgFeatPos[i+1] - avgFeatPos[i];
                //判断d1与d2方向是否一致（向量夹角小于一阈值），若一致，输出0
                float d1_mu , d2_mu;
                d1_mu = sqrt( d1.x * d1.x + d1.y * d1.y );
                d2_mu = sqrt( d2.x * d2.x + d2.y * d2.y );
                float yuxian = ( d1.x * d2.x + d1.y * d2.y ) / ( d1_mu * d2_mu );
                if( yuxian > cosinlim )
                {
                    shift_SD.push_back(0);
                }
                else
                {
                    Point2f d = d2 - d1;
                    d.x = d.x / 2.0f;
                    d.y = d.y / 2.0f;
                    float d_mu;
                    d_mu = sqrt( d.x * d.x + d.y * d.y );
                    shift_SD.push_back(d_mu);
                }
            }
            float avg_shift_SD = 0;
            //段内平均移动值
            for( int SD = 0 ; SD < shift_SD.size() ; SD++ )
            {
                avg_shift_SD += shift_SD[SD];
            }
            avg_shift_SD = avg_shift_SD / (float)shift_SD.size();
            /**/
            /*计算平均旋转振幅*/
            double avg_rotate_SD = 0;
            if( thetaVec.size() == ThreadContext::SEGSIZE - 1 )
            {
                vector<double> tmpRotate;//相邻两帧中前一帧到后一帧的旋转角度
                for( int i = 0 ; i < thetaVec.size() ; i++ )
                {
                    if( i == 0 )
                    {
                        tmpRotate.push_back( -thetaVec[i] );
                    }
                    else
                    {
                        if( i == thetaVec.size() - 1 )
                        {
                            tmpRotate.push_back( thetaVec[i] + thetaVec[i-1] );
                        }
                        else
                        {
                            tmpRotate.push_back( -thetaVec[i] + thetaVec[i-1] );
                        }
                    }
                }
                vector<double> rotate_SD;
                for( int i = 0 ; i < tmpRotate.size()-1 ; i++ )
                {
                    if( (tmpRotate[i] > 0 && tmpRotate[i+1] > 0) || (tmpRotate[i] < 0 && tmpRotate[i+1] < 0) )
                    {
                        rotate_SD.push_back(0);
                    }
                    else
                    {
                        rotate_SD.push_back(abs(tmpRotate[i]-tmpRotate[i+1]));
                    }
                }
                for( int SD = 0 ; SD < rotate_SD.size() ; SD++ )
                {
                    avg_rotate_SD += rotate_SD[SD];
                }
                avg_rotate_SD = avg_rotate_SD / (double)rotate_SD.size();
            }
            /**/

            if( avg_shift_SD < ThreadContext::TRANSLATE_AMPLITUDE )//认为稳定(平移振幅)
            {
                isStable = true;
            }
            if( thetaVec.size() == ThreadContext::SEGSIZE - 1 )
            {
                if( avg_rotate_SD > ThreadContext::ROTATE_AMPLITUDE )//认为不稳定(旋转振幅)
                {
                    isStable = false;
                }
            }
        }
    }

    //int x_tip = videoSize.width * ( 1 - cropRation ) / 2;
    //int y_tip = videoSize.height * ( 1 - cropRation ) / 2;
    //if( cropControlFlag )//如果进行裁剪控制
    //{
    //	Mat out = frameVec[start].rowRange( y_tip , videoSize.height - y_tip ).colRange( x_tip , videoSize.width - x_tip );
    //	stableVec.push_back(out);
    //}
    //else
    //{
    //	stableVec.push_back(frameVec[start]);
    //}

    Mat tmp = Mat::zeros(2,3,CV_64FC1);
    tmp.at<double>(0,0) = 1;
    tmp.at<double>(1,1) = 1;

    cv::Mat transVec3(3,3,CV_64F);
    for (int i = 0; i < 6; i ++) {
        transVec3.at<double>(i/3,i%3)=tmp.at<double>(i/3,i%3);
    }
    transVec3.at<double>(2,0)=0;
    transVec3.at<double>(2,1)=0;
    transVec3.at<double>(2,2)=1;
    ThreadContext::stableTransformVec[start] =transVec3*ThreadContext::stableRVec[start] ;
    //前关键帧到后关键帧的旋转角度
    double s2e_theta = 0;
    if( affineMatrix.size() != 0  )
    {
        if( !affineMatrix[affineMatrix.size()-1].empty() )
        {
            s2e_theta = thetaVec[thetaVec.size()-1];
        }
    }

    if( videoSize == Size(1920,1080) )//降采样后坐标为原坐标的1/2，因此要乘2恢复
    {
        for( int i = 0 ; i < avgFeatPos.size() ; i++ )
        {
//            avgFeatPos[i].x = avgFeatPos[i].x * 4;
//            avgFeatPos[i].y = avgFeatPos[i].y * 4;
            avgFeatPos[i].x = avgFeatPos[i].x * ThreadContext::DOWNSAMPLE_SCALE;
            avgFeatPos[i].y = avgFeatPos[i].y * ThreadContext::DOWNSAMPLE_SCALE;
        }
    }

    for( int m = 0 ; m < length - 2 ; m++ )//计算段内非关键帧的运动补偿
    {
        int index = ( start + m + 1 ) % ( ThreadContext::BUFFERSIZE );
        Point2f shift;//平移补偿
        Mat affine;//旋转补偿
        if( affineMatrix.size() == 0 )//即在此段内横跨整段的轨迹条数小于3
        {
            shift = Point2f(0.0,0.0);
            affine = Mat::zeros(2,3,CV_64FC1);
            affine.at<double>(0,0) = 1;
            affine.at<double>(1,1) = 1;
        }
        else
        {
            //平移补偿计算
            Point2f tmpP;
            tmpP = avgFeatPos[avgFeatPos.size()-1] - avgFeatPos[0];
            tmpP.x = ( m + 1 ) * tmpP.x / (float)( avgFeatPos.size() - 1 );
            tmpP.y = ( m + 1 ) * tmpP.y / (float)( avgFeatPos.size() - 1 );
            shift = avgFeatPos[0] - avgFeatPos[m+1] + tmpP;

            //旋转补偿计算
            double degree;
            if( affineMatrix[affineMatrix.size()-1].empty() || affineMatrix[m].empty() )
            {
                /*affine = Mat::zeros(2,3,CV_64FC1);
                affine.at<double>(0,0) = 1;
                affine.at<double>(1,1) = 1;*/
                degree = 0;
            }
            else
            {
                double delta_theta;
                delta_theta = ( m + 1 ) * s2e_theta / (double)( avgFeatPos.size() - 1 );

                //通过奇异值分解得到非关键帧到前关键帧的旋转矩阵
                double theta;
                //SVD thissvd;
                //Mat i2sAffine , W2 , U2 , VT2;
                //i2sAffine = affineMatrix[m].colRange(0,2);
                //thissvd.compute(i2sAffine,W2,U2,VT2,SVD::FULL_UV);//计算旋转矩阵
                //i2sAffine = U2*VT2;
                //theta = asinf( i2sAffine.at<double>(0,1) );
                theta = thetaVec[m];


                //计算旋转补偿量
                //double degree;//角度为正表示逆时针旋转
                degree = theta + delta_theta;
                //degree = degree * 180 / 3.1415926;
                //affine = getRotationMatrix2D( avgFeatPos[m+1]+shift , degree , 1 );
            }
//            degree=-degree;//todo get neg degree

            if( shakeDetect )
            {
                if( isStable )
                {
                    shift = Point2f(0.0,0.0);
                    degree = 0;
                }
            }

            if( cropControlFlag )
            {
                cropControl( cropRation , avgFeatPos[m+1] , shift , degree );
                degree = degree * 180 / 3.1415926;
                affine = getRotationMatrix2D( avgFeatPos[m+1]+shift , degree , 1 );
            }
            else
            {
                degree = degree * 180 / 3.1415926;
                affine = getRotationMatrix2D( avgFeatPos[m+1]+shift , degree , 1 );
            }
        }

        //平移补偿转化为齐次矩阵形式
        Mat shiftAffine = Mat::zeros(3,3,CV_64FC1);
        shiftAffine.at<double>(0,0) = 1;
        shiftAffine.at<double>(1,1) = 1;
        shiftAffine.at<double>(0,2) = shift.x;
        shiftAffine.at<double>(1,2) = shift.y;
        shiftAffine.at<double>(2,2) = 1;

        //旋转补偿转化为齐次矩阵形式
        Mat tmpaff = Mat::zeros(3,3,CV_64FC1);
        tmpaff.at<double>(0,0) = affine.at<double>(0,0);
        tmpaff.at<double>(0,1) = affine.at<double>(0,1);
        tmpaff.at<double>(0,2) = affine.at<double>(0,2);
        tmpaff.at<double>(1,0) = affine.at<double>(1,0);
        tmpaff.at<double>(1,1) = affine.at<double>(1,1);
        tmpaff.at<double>(1,2) = affine.at<double>(1,2);
        tmpaff.at<double>(2,2) = 1;

        Mat resultAffine;//最终补偿变换
        resultAffine = tmpaff * shiftAffine ;
        resultAffine = resultAffine.rowRange(0,2);

        //Mat out;//稳定帧
        //warpAffine( tmpimg , out , resultAffine , videoSize ,1,0,Scalar(0,0,0));
        //if( cropControlFlag )
        //{
        //	out = out.rowRange( y_tip , videoSize.height - y_tip ).colRange( x_tip , videoSize.width - x_tip );
        //}
        //stableVec.push_back(out);

        cv::Mat transVec3(3,3,CV_64F);
        for (int i = 0; i < 6; i ++) {
            transVec3.at<double>(i/3,i%3)=resultAffine.at<double>(i/3,i%3);
        }
        transVec3.at<double>(2,0)=0;
        transVec3.at<double>(2,1)=0;
        transVec3.at<double>(2,2)=1;
//        __android_log_print(ANDROID_LOG_ERROR, "NStableProcessor", "transVec3: %f",transVec3.at<double>(0,0));
//        __android_log_print(ANDROID_LOG_ERROR, "NStableProcessor", "stableRvec: %f",ThreadContext::stableRVec[index].at<double>(0,0));

        ThreadContext::stableTransformVec[index] = transVec3*ThreadContext::stableRVec[index];
    }

    ThreadContext::trj.pop_front();
}

//经过平移旋转后仍包括裁剪窗口返回true，否则返回false，且修改shift和degree值
bool ThreadCompensation::cropControl( float cropRation , Point2f center , Point2f &shift , double &degree )
{
    vector<Point2f> pt;//原始图像顶点坐标
    pt.push_back( Point2f(0,0) );
    pt.push_back( Point2f(0,videoSize.height-1) );
    pt.push_back( Point2f(videoSize.width-1,videoSize.height-1) );
    pt.push_back( Point2f(videoSize.width-1,0) );
    vector<Point2f> pt_crop;//裁剪窗口顶点坐标
    int x_tip = (int) (videoSize.width * (1 - cropRation ) / 2);
    int y_tip = (int) (videoSize.height * (1 - cropRation ) / 2);
    pt_crop.push_back( Point2f(x_tip,y_tip) );
    pt_crop.push_back( Point2f(x_tip,videoSize.height-y_tip) );
    pt_crop.push_back( Point2f(videoSize.width-x_tip,videoSize.height-y_tip) );
    pt_crop.push_back( Point2f(videoSize.width-x_tip,y_tip) );


    Mat shiftMat = Mat::eye(3,3,CV_64FC1);
    shiftMat.at<double>(0,2) = shift.x;
    shiftMat.at<double>(1,2) = shift.y;
    double angle = degree * 180 / 3.1415926;
    Mat tmpRotateMat = getRotationMatrix2D( center+shift , angle , 1 );
    Mat rotateMat = Mat::zeros(3,3,CV_64FC1);
    rotateMat.at<double>(0,0) = tmpRotateMat.at<double>(0,0);
    rotateMat.at<double>(0,1) = tmpRotateMat.at<double>(0,1);
    rotateMat.at<double>(0,2) = tmpRotateMat.at<double>(0,2);
    rotateMat.at<double>(1,0) = tmpRotateMat.at<double>(1,0);
    rotateMat.at<double>(1,1) = tmpRotateMat.at<double>(1,1);
    rotateMat.at<double>(1,2) = tmpRotateMat.at<double>(1,2);
    rotateMat.at<double>(2,2) = 1;
    Mat affine = rotateMat * shiftMat;
    affine = affine.rowRange(0,2);

    if( isInsideAfterTransform( affine , pt_crop , pt ) )//经过平移旋转后依然包括裁剪窗口
    {
        return true;
    }
    else
    {
        if( x_tip < abs(shift.x) || y_tip < abs(shift.y) )//经过平移后不包括裁剪窗口
        {
            float ratio1 = (float)x_tip / abs(shift.x);
            float ratio2 = (float)y_tip / abs(shift.y);
            if( ratio1 < ratio2 )
            {
                if( shift.x > 0 )
                {
                    shift.x = x_tip;
                }
                else
                {
                    shift.x = -x_tip;
                }
                shift.y = ratio1 * shift.y;
            }
            else
            {
                shift.x = ratio2 * shift.x;
                if( shift.y > 0 )
                {
                    shift.y = y_tip;
                }
                else
                {
                    shift.y = -y_tip;
                }
            }
            degree = 0;
        }
        else//经过平移后包括裁剪窗口
        {
            /*计算最大旋转角*/
            vector<Point2f> new_crop_pt;
            for( int i = 0 ; i < 4 ; i++ )
            {
                new_crop_pt.push_back(pt_crop[i]-shift);
            }

            double maxDegree[4];
            vector<Point2f> img_line;
            vector<Point2f> crop_line;
            img_line.push_back( Point2f(0,0) );
            img_line.push_back( Point2f(videoSize.width-1,0) );
            crop_line.push_back( new_crop_pt[0] );
            crop_line.push_back( new_crop_pt[3] );
            maxDegree[0] = computeMaxDegree( img_line , crop_line , degree , center );

            img_line[1] = Point2f(videoSize.height-1,0);
            crop_line[0] = Point2f(videoSize.height-1-new_crop_pt[1].y , new_crop_pt[1].x);
            crop_line[1] = Point2f(videoSize.height-1-new_crop_pt[0].y , new_crop_pt[0].x);
            Point2f newCenter;
            newCenter.x = videoSize.height-1-center.y;
            newCenter.y = center.x;
            maxDegree[1] = computeMaxDegree( img_line , crop_line , degree , newCenter );

            img_line[1] = Point2f(videoSize.width-1,0);
            crop_line[0] = Point2f(videoSize.width-1-new_crop_pt[2].x , videoSize.height-1-new_crop_pt[2].y);
            crop_line[1] = Point2f(videoSize.width-1-new_crop_pt[1].x , videoSize.height-1-new_crop_pt[1].y);
            newCenter.x = videoSize.width-1-center.x;
            newCenter.y = videoSize.height-1-center.y;
            maxDegree[2] = computeMaxDegree( img_line , crop_line , degree , newCenter );

            img_line[1] = Point2f(videoSize.height-1,0);
            crop_line[0] = Point2f(new_crop_pt[3].y , videoSize.width-1-new_crop_pt[3].x);
            crop_line[1] = Point2f(new_crop_pt[2].y , videoSize.width-1-new_crop_pt[2].x);
            newCenter.x = center.y;
            newCenter.y = videoSize.width-1-center.x;
            maxDegree[3] = computeMaxDegree( img_line , crop_line , degree , newCenter );

            if( degree > 0 )
            {
                double min = degree;
                for( int i = 0 ; i < 4 ; i++ )
                {
                    if( min > maxDegree[i] )
                    {
                        min = maxDegree[i];
                    }
                }
                degree = min;
            }
            else
            {
                double max = degree;
                for( int i = 0 ; i < 4 ; i++ )
                {
                    if( max < maxDegree[i] )
                    {
                        max = maxDegree[i];
                    }
                }
                degree = max;
            }
            /**/
        }

        return false;
    }
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