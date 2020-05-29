package me.zhehua.gryostable.widget;

import android.content.Context;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.util.Log;
import android.view.SurfaceHolder;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

import me.zhehua.gryostable.CameraBridgeViewBase;
import me.zhehua.gryostable.StableProcessor;
import me.zhehua.gryostable.util.OnRecordListener;

public class GlRenderView extends GLSurfaceView {
    private String TAG = "GlRenderView";
    private String savePath;
    private OnRecordListener onRecordListener;
    public GlRenderWrapper glRender;
    public final Object syncObj = new Object();
    public StableProcessor stableProcessor;
    public int mFrameHeight = 1080;
    public int mFrameWidth = 1920;


    public GlRenderView(Context context){
        this(context, null);
    }
    public GlRenderView(Context context, AttributeSet attrs){
        super(context, attrs);
        setEGLContextClientVersion(3);
        glRender = new GlRenderWrapper(this);
        setRenderer(glRender);
        setRenderMode(RENDERMODE_WHEN_DIRTY);
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        super.surfaceDestroyed(holder);
        glRender.onSurfaceDestory();
    }

    public void startRecord(){
        float speed = 1.0f;
        glRender.startRecord(speed, savePath);
    }
    public void stopRecord() {
        glRender.stopRecord();
    }
    public void setSavePath(String savePath) {
        this.savePath = savePath;
    }
    public void setOnRecordListener(OnRecordListener onRecordListener) {
        this.onRecordListener = onRecordListener;
        glRender.setOnRecordListener(onRecordListener);
    }

    public void startDisplayThread() {
        new Thread(new GlRenderView.DisplayThread()).start();
    }
    public class DisplayThread implements Runnable{

        Mat transMat = new Mat(3, 3, CvType.CV_64F);
        Mat framePic = new Mat(mFrameHeight, mFrameWidth, CvType.CV_8UC3);
        double[] transData = new double[9];
        float[] transDataF = new float[9];
        Mat rsMat = new Mat(30, 3, CvType.CV_64F);

        @Override
        public void run() {
            Thread.currentThread().setName("getTransVec Thread");

            while (true){
                synchronized (syncObj){
                    try{
                        syncObj.wait(30);
                    } catch (InterruptedException e){
                        e.printStackTrace();
                    }
                }

                if(stableProcessor != null){

                    stableProcessor.dequeueOutputBuffer(transMat, framePic, rsMat);
                    Log.d(TAG, "run+++++: "+rsMat.dump());
                }

                if(framePic.empty()){
                    Log.i(TAG, "yes, get empty output Mat");
                    break;
                }
                if (transMat.type() != CvType.CV_64F) {
                    Log.e(TAG, "WTF?? " + transMat.dump() + framePic.dump());
                    Log.e(TAG, transMat + " " + transMat);
                }
                transMat.get(0 ,0 , transData);
                for (int i = 0; i < 9; i++) {
                    transDataF[i] = (float) transData[i];
                }

//                glRender.cameraFilter.transformMatrix = transDataF;
                glRender.screenFilter.transformMatrix = transDataF;
//                glRender.cameraFilter.outputMat = framePic;
                glRender.screenFilter.outputMat = framePic;
                glRender.screenFilter.rsMat = rsMat;

                if(glRender.isready){
                    GlRenderView.this.requestRender();
                }


                if(stableProcessor != null){
                    stableProcessor.enqueueOutputBuffer();
                }



            }
        }
    }
}
