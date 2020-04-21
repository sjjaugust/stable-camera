//
// Created by ShiJJ on 2020/4/16.
//
#include <jni.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include "new_processor/ThetaHelper.h"
#include "new_processor/StableProcessor.h"

ThetaHelper n_th;
StableProcessor n_sp;

//TODO 转化之后乘一下
extern "C"
JNIEXPORT void JNICALL
Java_me_zhehua_gryostable_ThetaHelper_n_1sensor_1changed(JNIEnv *env, jobject instance,
                                                         jlong timestamp, jfloat x, jfloat y,
                                                         jfloat z){
    n_th.PutValue(timestamp / 1000000000.0, x, y, z);
}
extern "C"
JNIEXPORT void JNICALL
Java_me_zhehua_gryostable_ThetaHelper_n_1init(JNIEnv *env, jobject instance){
    n_th.Init();
}
extern "C"
JNIEXPORT void JNICALL
Java_me_zhehua_gryostable_ThetaHelper_n_1getQuaternion(JNIEnv *env, jobject instance,
                                                       jlong timestamp, jlong out_mat){
    n_th.GetQuaternion(timestamp / 1000000000.0, (cv::Mat*)out_mat);
}
extern "C"
JNIEXPORT jint JNICALL
Java_me_zhehua_gryostable_StableProcessor_n_1dequeueInputBuffer(JNIEnv *env, jobject instance){
    return  n_sp.DequeueInputBuffer();
}
extern  "C"
JNIEXPORT void JNICALL
Java_me_zhehua_gryostable_StableProcessor_n_1enqueueInputBuffer(JNIEnv *env, jobject instance,
                                                                jint buffer_index, jlong new_frame,
                                                                jlong quaternion_mat){
    n_sp.EnqueueInputBuffer(buffer_index, (cv::Mat*)new_frame, (cv::Mat*)quaternion_mat);
}
extern  "C"
JNIEXPORT void JNICALL
Java_me_zhehua_gryostable_StableProcessor_n_1enqueueOutputBuffer(JNIEnv *env, jobject instance){
    n_sp.EnqueueOutputBuffer();
}
extern  "C"
JNIEXPORT void JNICALL
Java_me_zhehua_gryostable_StableProcessor_n_1dequeueOutputBuffer(JNIEnv *env, jobject instance,
                                                                jlong stableVec, jlong frame){
    n_sp.DequeueOutputBuffer((cv::Mat*)stableVec, (cv::Mat*)frame);
}
extern  "C"
JNIEXPORT void JNICALL
Java_me_zhehua_gryostable_StableProcessor_setCrop(JNIEnv *env, jobject instance, jboolean is_crop){
    n_sp.SetCrop(is_crop);
}
extern  "C"
JNIEXPORT jlong JNICALL
Java_me_zhehua_gryostable_StableProcessor_n_1StableProcessor(JNIEnv *env, jobject instance){
    return (jlong) &n_sp;
}
extern  "C"
JNIEXPORT void JNICALL
Java_me_zhehua_gryostable_StableProcessor_n_1Init(JNIEnv *env, jobject instance){
    n_sp.Init();
}