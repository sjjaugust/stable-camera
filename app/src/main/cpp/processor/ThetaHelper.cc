//
// Created by 张哲华 on 2019/3/1.
//

#include "ThetaHelper.h"
#include <android/log.h>


Mat ThetaHelper::getRR(Mat oldRotation, Mat newRotation)
{
    cv::Mat a1 = (cv::Mat_<double>(3, 3) << 0.0,1.0,0.0, -1.0,0.0,0.0, 0.0 ,0.0 ,1.0);
    cv::Mat a2 = (cv::Mat_<double>(3, 3) << 1,0,0, 0,-1,0, 0 ,0 ,-1);
    cv::Mat A=a1*a2;
    cv::Mat hom = cv::Mat::eye(cv::Size(3,3),CV_64F);

    cv::Mat sta=oldRotation.t()*newRotation;
    cv::Mat temp=A.t()*sta.t()*hom.t()*A;
    cv::Mat R=inmat*temp*inmat.inv();
    __android_log_print(ANDROID_LOG_ERROR, "ThetaHelper", "RRRRRR:%f", oldRotation.at<double>(0,1));
    // warpPerspective(oldImage, newImage, RR, oldImage.size(),cv::INTER_CUBIC);//INTER_LINEAR);
    return R;
}


cv::Vec<double, 3> ThetaHelper::getTheta()
    {
        cv::Vec<double, 3> theta;
        for(int i=0; i<theta.rows;i++)
            theta[i] = 0;

        //获取第一个帧时间戳和陀螺仪时间戳
        double gtime= Timeg[gyindex];
        double ftime= Timeframe[findex];

        if(findex>0)
        {
            double lastftime= Timeframe[findex - 1];
            theta[0]=-lastx*(gtime-lastftime)+lastt[0];
            theta[1]=-lasty*(gtime-lastftime)+lastt[1];
            theta[2]=-lastz*(gtime-lastftime)+lastt[2];


            // NSLog(@"frame:%f",lastftime);
            //NSLog(@"gyro:%f",gtime);
        }


        while(gtime<ftime)
        {

            double anglevx= roxl[gyindex];
            double anglevy= royl[gyindex];
            double anglevz= rozl[gyindex];
            double gtimenext= Timeg[gyindex + 1];
            //  double time2=[[Timeframe objectAtIndex:findex] doubleValue];
            if(gtimenext<ftime)
            {
                // double gtimenext=[[Timeg objectAtIndex:gyindex+1] doubleValue];
                theta[0]=theta[0]+(-anglevx)*(gtimenext-gtime);
                theta[1]=theta[1]+(-anglevy)*(gtimenext-gtime);
                theta[2]=theta[2]+(-anglevz)*(gtimenext-gtime);
//                __android_log_print(ANDROID_LOG_ERROR, "ThetaHelper", "thththth:%f", theta[0]);
                gtime=gtimenext;
                gyindex++;
            }
            else
            {
                theta[0]=theta[0]+(-anglevx)*(ftime-gtime);
                lastx=anglevx;
                theta[1]=theta[1]+(-anglevy)*(ftime-gtime);
                lasty=anglevy;
                theta[2]=theta[2]+(-anglevz)*(ftime-gtime);
                lastz=anglevz;
                gyindex++;
                break;
            }
        }


        findex++;
        lastt=theta;
        // NSLog(@"old  x:%f,y:%f,z:%f,",theta[0],theta[1],theta[2]);
        return theta;

}

cv::Vec<double, 3> ThetaHelper::getNewTheta(cv::Vec<double, 3> oldtheta)
{
    //卡尔曼滤波

    cv::Vec<double, 3> newtheta;
    cv::Vec<double, 3> ptheta;

    if(findex==1)
    {
        lasttheta=oldtheta;
        newtheta=oldtheta;
    }
    else
    {
        if(angledex>20)
        {
            double xqian= oldx[angledex - 20];
            double xhou= oldx[angledex - 1];
            double tx=xhou-xqian;
            rx=abs(5/pow(tx, 2));
            if(rx>exp(5))
                rx=exp(5);
            double yqian= oldy[angledex - 20];
            double yhou= oldy[angledex - 1];
            double ty=yhou-yqian;
            ry=abs(5/pow(ty, 2));
            if(ry>exp(5))
                ry=exp(5);
            double zqian= oldz[angledex - 20];
            double zhou= oldz[angledex - 1];
            double tz=zhou-zqian;
            rz=abs(5/pow(tz, 2));
            if(rz>exp(5))
                rz=exp(5);
        }
        ptheta=lasttheta;
        double ppx=px+q;//10+exp(-3)=10.0498
        double ppy=py+q;
        double ppz=pz+q;
        //ppx为上一帧的总体误差，rx为实际值的误差
        kx=ppx/(ppx+rx);
        ky=ppy/(ppy+ry);
        kz=ppz/(ppz+rz);
        __android_log_print(ANDROID_LOG_ERROR, "ThetaHelper", "rxyz: %f, %f, %f",rx,ry,rz);
        //计算k时刻实际值,ptheta为预测值也就是上一个角度值，k为卡尔曼增益，oldtheta为实际值
        newtheta[0]=ptheta[0]+kx*(oldtheta[0]-ptheta[0]);
        newtheta[1]=ptheta[1]+ky*(oldtheta[1]-ptheta[1]);
        newtheta[2]=ptheta[2]+kz*(oldtheta[2]-ptheta[2]);
        //更新px
        px=(1-kx)*ppx;
        py=(1-ky)*ppy;
        pz=(1-kz)*ppz;
    }
    lasttheta=newtheta;

    return newtheta;

}

cv::Mat ThetaHelper::getRotationMat(cv::Vec<double, 3> theta)
{
    cv::Mat cvmat(3, 3, CV_64F);
    cv::Mat skew_mat(3, 3, CV_64F);
    int eq=0;
    double th=theta[0]*theta[0]+theta[1]*theta[1]+theta[2]*theta[2];
    th=sqrt(th);
//    __android_log_print(ANDROID_LOG_ERROR, "ThetaHelper", "thththth:%f", th);
    if(th==0)
        eq=1;
    th=th+eq;
    theta[0]=theta[0]/th;
    theta[1]=theta[1]/th;
    theta[2]=theta[2]/th;
    //cout<<w<<' '<<x<<' '<<y<<' '<<z<<endl;
    //计算旋转矩阵
    skew_mat.at<double>(0,0)=0;
    skew_mat.at<double>(0,1)=-theta[2];
    skew_mat.at<double>(0,2)=theta[1];
    skew_mat.at<double>(1,0)=theta[2];
    skew_mat.at<double>(1,1)=0;
    skew_mat.at<double>(1,2)=-theta[0];
    skew_mat.at<double>(2,0)=-theta[1];
    skew_mat.at<double>(2,1)=theta[0];
    skew_mat.at<double>(2,2)=0;
    cv::Mat e=cv::Mat::eye(3,3,CV_64F);
    cvmat =e+sin(th)*skew_mat.t()+(1-cos(th))*(skew_mat*skew_mat).t();
    return cvmat;
}

void ThetaHelper::init() {
    c = 0;
    gyindex=0;
    findex=0;
    angledex=-1;
    lastx=0;
    lasty=0;
    lastz=0;
    px=10;
    py=10;
    pz=10;
    q=exp(-3);
    rx=exp(5);
    ry=exp(5);
    rz=exp(5);

    //果冻效应相关
    rsFrameIndex = 0;
    rsGyroIndex = 0;
    rsLastx = 0;
    rsLasty = 0;
    rsLastz = 0;
}

bool ThetaHelper::isInside(cv::Mat cropvertex, cv::Mat newvertex)
{
    bool aInside = true;
    for( int i = 0 ; i < 4 ; i++ )
    {
        for( int j = 0 ; j < 4 ; j++ )
        {
            Point2f vec1 , vec2;
            vec1.x=float(newvertex.at<double>(0,j)-cropvertex.at<double>(0,i));
            vec1.y=float(newvertex.at<double>(1,j)-cropvertex.at<double>(1,i));
            vec2.x=float(newvertex.at<double>(0,(j+1)%4)-newvertex.at<double>(0,j));
            vec2.y=float(newvertex.at<double>(1,(j+1)%4)-newvertex.at<double>(1,j));

            float cross_product = vec1.x * vec2.y - vec2.x * vec1.y;
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

void ThetaHelper::cropControl(Mat& transVec) {
    cv::Mat newvertex=transVec*vertex;
    newvertex.at<double>(0,0)=newvertex.at<double>(0,0)/newvertex.at<double>(2,0);
    newvertex.at<double>(1,0)=newvertex.at<double>(1,0)/newvertex.at<double>(2,0);

    newvertex.at<double>(0,1)=newvertex.at<double>(0,1)/newvertex.at<double>(2,1);
    newvertex.at<double>(1,1)=newvertex.at<double>(1,1)/newvertex.at<double>(2,1);

    newvertex.at<double>(0,2)=newvertex.at<double>(0,2)/newvertex.at<double>(2,2);
    newvertex.at<double>(1,2)=newvertex.at<double>(1,2)/newvertex.at<double>(2,2);

    newvertex.at<double>(0,3)=newvertex.at<double>(0,3)/newvertex.at<double>(2,3);
    newvertex.at<double>(1,3)=newvertex.at<double>(1,3)/newvertex.at<double>(2,3);

    bool allInside = isInside(cropvertex, newvertex);
    double ratio=0.99;
    cv::Mat resulttemp = transVec.clone();
    while((!allInside)&&(ratio>=0))
    {

        double transdet = cv::determinant(transVec);
        cv::Mat transtemp = transVec/pow(transdet, 1.0/3);
        resulttemp = I*(1-ratio)+transtemp*ratio;

        ratio=ratio-0.01;
        newvertex=resulttemp*vertex;
        newvertex.at<double>(0,0)=newvertex.at<double>(0,0)/newvertex.at<double>(2,0);
        newvertex.at<double>(1,0)=newvertex.at<double>(1,0)/newvertex.at<double>(2,0);

        newvertex.at<double>(0,1)=newvertex.at<double>(0,1)/newvertex.at<double>(2,1);
        newvertex.at<double>(1,1)=newvertex.at<double>(1,1)/newvertex.at<double>(2,1);

        newvertex.at<double>(0,2)=newvertex.at<double>(0,2)/newvertex.at<double>(2,2);
        newvertex.at<double>(1,2)=newvertex.at<double>(1,2)/newvertex.at<double>(2,2);

        newvertex.at<double>(0,3)=newvertex.at<double>(0,3)/newvertex.at<double>(2,3);
        newvertex.at<double>(1,3)=newvertex.at<double>(1,3)/newvertex.at<double>(2,3);

        allInside = isInside(cropvertex, newvertex);
    }

    resulttemp.copyTo(transVec);

}

void ThetaHelper::getR(double timestamp, Mat *matR ,bool isCrop) {
    rsGyroIndex = gyindex;

    Timeframe.push_back(timestamp);
    cv::Vec<double, 3> oldtheta=getTheta();
    oldx.push_back(oldtheta[0]);//[oldx addObject:[NSNumber numberWithDouble: oldtheta[0]]];
    oldy.push_back(oldtheta[1]);//[oldy addObject:[NSNumber numberWithDouble: oldtheta[1]]];
    oldz.push_back(oldtheta[2]);//[oldz addObject:[NSNumber numberWithDouble: oldtheta[2]]];
    angledex++;
    cv::Vec<double, 3> newtheta=getNewTheta(oldtheta);//[self getNewTheta:oldtheta];

    cv::Mat oldRotation=getRotationMat(oldtheta);//[self getRotationMat:oldtheta];
    cv::Mat newRotation=getRotationMat(newtheta);//[self getRotationMat:newtheta];
//    __android_log_print(ANDROID_LOG_ERROR, "ThetaHelper", "old: %f",oldRotation.at<double>(0,0));
//    __android_log_print(ANDROID_LOG_ERROR, "ThetaHelper", "new: %f",newRotation.at<double>(0,0));
    RR=getRR(oldRotation, newRotation);//[self getRR:oldRotation :newRotation];
    if (isCrop) {
        cropControl(RR);
    }
    __android_log_print(ANDROID_LOG_ERROR, "ThetaHelper", "RR: %f",RR.at<double>(0,0));

    RR.copyTo(*matR);

    //果冻效应相关
    m_RsGyroTheta = getRsTheta();
    rsGyroThetaRows = m_RsGyroTheta.size();
}

vector<cv::Vec<double, 4>> ThetaHelper::getRsTheta(){

    cv::Mat gyroTheta(3, 4, CV_64F);
    vector<cv::Vec<double, 4>> rsGyroTheta;

    double frameTime = Timeframe[rsFrameIndex];
    double gyroTime = Timeg[rsGyroIndex];
    cv::Vec<double, 4> temp;
    for(int i = 0; i < 4; i++){
        temp[i] = 0;
    }
    if(rsFrameIndex>0){
        double lastftime= Timeframe[rsFrameIndex - 1];
        temp[0]=0;
        temp[1]=-rsLastx*(gyroTime-lastftime)+rsLastt[1];
        temp[2]=-rsLasty*(gyroTime-lastftime)+rsLastt[2];
        temp[3]=-rsLastz*(gyroTime-lastftime)+rsLastt[3];
    }
    while (gyroTime<frameTime){
        double angleX = roxl[rsGyroIndex];
        double angleY = royl[rsGyroIndex];
        double angleZ = rozl[rsGyroIndex];
        double gyroTimeNext = Timeg[rsGyroIndex+1];
//
        if(gyroTimeNext<frameTime){
            temp[0] = gyroTimeNext;
            temp[1] = temp[1]+(gyroTimeNext-gyroTime)*(-angleX);
            temp[2] = temp[2]+(gyroTimeNext-gyroTime)*(-angleY);
            temp[3] = temp[3]+(gyroTimeNext-gyroTime)*(-angleZ);
            rsGyroTheta.push_back(temp);
            rsGyroIndex++;
//            __android_log_print(ANDROID_LOG_ERROR, "ThetaHelper", "thththth:%d %d", rsFrameIndex, rsGy);
        } else{
            temp[0] = frameTime;
            temp[1] = temp[1]+(frameTime-gyroTime)*(-angleX);
            temp[2] = temp[2]+(frameTime-gyroTime)*(-angleY);
            temp[3] = temp[3]+(frameTime-gyroTime)*(-angleZ);
            rsGyroTheta.push_back(temp);

            rsLastx = angleX;
            rsLasty = angleY;
            rsLastz = angleZ;
            rsLastt = temp;
            rsGyroIndex++;
            break;
        }
//        rsGyroIndex++;
        gyroTime = Timeg[rsGyroIndex];
    }
//    __android_log_print(ANDROID_LOG_ERROR, "ThetaHelper" ,"gyroTheta: %d %d", rsFrameIndex, rsGyroTheta.size());
    rsFrameIndex++;
    return rsGyroTheta;

//    __android_log_print(ANDROID_LOG_ERROR, "ThetaHelper" ,"gyroTheta: %f", gyroTheta.at<double>(0,1));

}

void ThetaHelper::getRsThetaRows(int rows) {
    rows = rsGyroThetaRows;
//    __android_log_print(ANDROID_LOG_ERROR, "ThetaHelper" ,"gyroTheta: %d", rows);
}
void ThetaHelper::rsChangeVectorToMat(cv::Mat* rsOutMat) {
    //将vector转化为mat
    cv::Mat temp(m_RsGyroTheta.size(), 4, CV_64F);
    for(int i = 0; i < m_RsGyroTheta.size(); i++){
        for(int j = 0; j < 4; j++){
            temp.at<double>(i, j) = m_RsGyroTheta[i][j];
        }
    }
    temp.copyTo(*rsOutMat);
}

void ThetaHelper::putValue(double timestamp, float x, float y, float z) {
    Timeg.push_back(timestamp);

    roxl.push_back(y);
    royl.push_back(-x);
    rozl.push_back(z);
}

