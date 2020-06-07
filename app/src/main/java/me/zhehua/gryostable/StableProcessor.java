package me.zhehua.gryostable;

/**
 * Created by zhehua on 24/10/2017.
 */


import org.opencv.core.Mat;

/**
 * Created by zhehua on 23/09/2017.
 */

public class StableProcessor {
    public final long nativeObj;

    public StableProcessor(long nativeObj_) {
        if (nativeObj_ == 0)
            throw new UnsupportedOperationException("StableProcessor Native object address is NULL");
        nativeObj = nativeObj_;
    }

    public StableProcessor() {
        nativeObj = n_StableProcessor();
    }

    public void init(int width, int height) {
        n_Init(width, height);
    }

    public int dequeueInputBuffer() {
        return n_dequeueInputBuffer();
    }

    public void enqueueInputBuffer(int bufIdx, Mat newFrame, Mat RR, Mat rs_out_mat) {
        n_enqueueInputBuffer(bufIdx, newFrame.nativeObj, RR.nativeObj, rs_out_mat.nativeObj);
    }

    public void enqueueOutputBuffer() {
        n_enqueueOutputBuffer();
    }

    public void dequeueOutputBuffer(Mat stableVec, Mat frame, Mat rsMat) {
        n_dequeueOutputBuffer(stableVec.nativeObj, frame.nativeObj, rsMat.nativeObj);
    }

    public void setCrop(boolean isCrop) {
        n_setCrop(isCrop);
    }

    public void setDrawStatus(boolean isDraw){
        n_setDrawStatus(isDraw);
    }

    private native long n_StableProcessor();
    private native void n_Init(int width, int height);
    private native int n_dequeueInputBuffer();
    private native void n_enqueueInputBuffer(int buffer_index, long new_frame, long RR, long rs_out_mat);
    private native void n_enqueueOutputBuffer();
    private native void n_dequeueOutputBuffer(long stableVec, long frame, long rsMat);
    private native void n_setCrop(boolean isCrop);
    private native void n_setDrawStatus(boolean isDraw);
}
