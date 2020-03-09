#include <jni.h>
#include "opencv2/opencv.hpp"
#include "processor/StableProcessor.h"
#include "processor/ThetaHelper.h"
#include <android/sensor.h>
#include <android/log.h>
#include <ctime>

using namespace cv;
using namespace threads;


#define  LOG_TAG    "accelerometergraph"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

extern "C" {

StableProcessor n_sp;
ThetaHelper n_th;


JNIEXPORT jlong JNICALL
Java_me_zhehua_gryostable_StableProcessor_n_1StableProcessor(JNIEnv *env, jobject instance) {
    return (jlong) &n_sp;
}

JNIEXPORT void JNICALL
Java_me_zhehua_gryostable_StableProcessor_n_1Init(JNIEnv *env, jobject instance, jint width,
                                                      jint height) {
    n_sp.Init(Size(width, height));
}

JNIEXPORT jint JNICALL
Java_me_zhehua_gryostable_StableProcessor_n_1dequeueInputBuffer(JNIEnv *env, jobject instance) {
    return n_sp.dequeueInputBuffer();
}
static Mat RR2stableVec = (cv::Mat_<double>(3, 3)<<0.0, 1.0, 0.0, -1.0, 0.0, 1080.0, 0.0, 0.0, 1.0);
static Mat stableVec2RR = (cv::Mat_<double>(3, 3)<<0.0, -1.0, 1080.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);

JNIEXPORT void JNICALL
Java_me_zhehua_gryostable_StableProcessor_n_1enqueueInputBuffer(JNIEnv *env, jobject instance,
                                                                    jint buffer_index,
                                                                    jlong new_frame,
                                                                    jlong RR,
                                                                    jlong rsOutTheta) {
    Mat* inFrame = (Mat*) new_frame;
    Mat* R = (Mat*) RR;
    *R = RR2stableVec * (*R) * stableVec2RR;
    n_sp.enqueueInputBuffer(buffer_index, inFrame, R, (Mat*)rsOutTheta);
}

JNIEXPORT void JNICALL
Java_me_zhehua_gryostable_StableProcessor_n_1enqueueOutputBuffer(JNIEnv *env,
                                                                      jobject instance) {
    n_sp.enqueueOutputBuffer();
}

JNIEXPORT void JNICALL
Java_me_zhehua_gryostable_StableProcessor_n_1dequeueOutputBuffer(JNIEnv *env, jobject instance,
                                                                     jlong stableVec, jlong frame, jlong rsMat) {
    Mat* nStableVec = (Mat*) stableVec;
    Mat* nFrame = (Mat*) frame;
    Mat* nRsMat = (Mat*) rsMat;
    n_sp.dequeueOutputBuffer(nStableVec, nFrame, nRsMat);
}

}



extern "C"
JNIEXPORT void JNICALL
Java_me_zhehua_gryostable_ThetaHelper_n_1getR(JNIEnv *env, jobject instance, jlong timestamp,
                                              jlong matR, jlong rsOutTheta, jboolean isCrop) {

    n_th.getR(timestamp / 1000000000.0, (Mat*) matR, (Mat*) rsOutTheta, isCrop);
}

extern "C"
JNIEXPORT void JNICALL
Java_me_zhehua_gryostable_ThetaHelper_n_1sensor_1changed(JNIEnv *env, jobject instance,
                                                         jlong timestamp, jfloat x, jfloat y,
                                                         jfloat z) {

    n_th.putValue(timestamp / 1000000000.0, x, y, z);
}

extern "C"
JNIEXPORT void JNICALL
Java_me_zhehua_gryostable_ThetaHelper_n_1init(JNIEnv *env, jobject instance) {
    // TODO time check
    time_t now = time(0);
    n_th.init();
    /*if (now < 1561910400) { // 2019.7.1 0:0:0

    }*/
}