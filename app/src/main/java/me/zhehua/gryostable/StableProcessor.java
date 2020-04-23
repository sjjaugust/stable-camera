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

    public void init() {
        n_Init();
    }

    public int dequeueInputBuffer() {
        return n_dequeueInputBuffer();
    }

    public void enqueueInputBuffer(int bufIdx, Mat newFrame, Mat out_mat) {
        n_enqueueInputBuffer(bufIdx, newFrame.nativeObj, out_mat.nativeObj);
    }

    public void enqueueOutputBuffer() {
        n_enqueueOutputBuffer();
    }

    public void dequeueOutputBuffer(Mat stableVec, Mat frame, Mat rs_convert_mat) {
        n_dequeueOutputBuffer(stableVec.nativeObj, frame.nativeObj, rs_convert_mat.nativeObj);
    }


    private native long n_StableProcessor();
    private native void n_Init();
    private native int n_dequeueInputBuffer();
    private native void n_enqueueInputBuffer(int buffer_index, long new_frame, long RR);
    private native void n_enqueueOutputBuffer();
    private native void n_dequeueOutputBuffer(long stableVec, long frame, long rs_convert_mat);
    public native void setCrop(boolean is_crop);
}
