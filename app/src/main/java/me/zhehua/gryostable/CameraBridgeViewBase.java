package me.zhehua.gryostable;

import android.content.Context;
import android.content.res.TypedArray;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Matrix;
import android.graphics.Rect;
import android.media.MediaCodec;
import android.media.MediaCodecInfo;
import android.media.MediaFormat;
import android.opengl.GLSurfaceView;
import android.os.Build;
import android.os.Environment;
import android.support.annotation.NonNull;
import android.util.AttributeSet;
import android.util.Log;
import android.view.Surface;
import android.view.SurfaceHolder;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.util.concurrent.Semaphore;

/**
 * This is a basic class, implementing the interaction with Camera and OpenCV library.
 * The main responsibility of it - is to control when camera can be enabled, process the frame,
 * call external listener to make any adjustments to the frame and then draw the resulting
 * frame to the screen.
 * The clients shall implement CvCameraViewListener.
 */
public class CameraBridgeViewBase extends GLSurfaceView {

    private static final String TAG = "CameraBridge";
    private static final int MAX_UNSPECIFIED = -1;
    private static final int STOPPED = 0;
    private static final int STARTED = 1;

    private int mState = STOPPED;
    private Bitmap mCacheBitmap;
    private SurfaceHolder.Callback mListener;
    private boolean mSurfaceExist;
    private Object mSyncObject = new Object();
    protected int mFrameOffsetX;
    protected int mFrameOffsetY;
    protected int mFrameWidth;
    protected int mFrameHeight;
    protected int mMaxHeight;
    protected int mMaxWidth;
    protected float mScale = 0;
    protected int mPreviewFormat = RGBA;
    protected int mCameraIndex = CAMERA_ID_ANY;
    protected boolean mEnabled;
    protected StableProcessor stableProcessor;
    CameraViewRenderer renderer;
    protected final Object syncObj = new Object();

    public static final int CAMERA_ID_ANY   = -1;
    public static final int CAMERA_ID_BACK  = 99;
    public static final int CAMERA_ID_FRONT = 98;
    public static final int RGBA = 1;
    public static final int GRAY = 2;

    // for video recording
    protected MediaCodec mediaCodec;
    FileChannel outChannel;
    protected Surface outSurface;
    protected Rect surfaceRect = new Rect(0, 0, 720, 1280);

    public CameraBridgeViewBase(Context context, int cameraId) {
        super(context);
        mCameraIndex = cameraId;
        getHolder().addCallback(this);
        mMaxWidth = MAX_UNSPECIFIED;
        mMaxHeight = MAX_UNSPECIFIED;
    }

    public CameraBridgeViewBase(Context context, AttributeSet attrs) {
        super(context, attrs);

        int count = attrs.getAttributeCount();
        Log.d(TAG, "Attr count: " + Integer.valueOf(count));

        TypedArray styledAttrs = getContext().obtainStyledAttributes(attrs, org.opencv.R.styleable.CameraBridgeViewBase);
//        if (styledAttrs.getBoolean(org.opencv.R.styleable.CameraBridgeViewBase_show_fps, false))
//            enableFpsMeter();

        mCameraIndex = styledAttrs.getInt(org.opencv.R.styleable.CameraBridgeViewBase_camera_id, -1);

        mMaxWidth = MAX_UNSPECIFIED;
        mMaxHeight = MAX_UNSPECIFIED;
        styledAttrs.recycle();
        renderer = new CameraViewRenderer(context);
        setEGLContextClientVersion(3);
        setRenderer(renderer);
        setRenderMode(RENDERMODE_WHEN_DIRTY);
    }

    void setListener(SurfaceHolder.Callback listener) {
        mListener = listener;
        getHolder().addCallback(listener);
    }

    public void initCodec() {
        MediaFormat mediaFormat = MediaFormat.createVideoFormat(MediaFormat.MIMETYPE_VIDEO_AVC, 720, 1280);
        mediaFormat.setInteger(MediaFormat.KEY_FRAME_RATE, 30);
        mediaFormat.setInteger(MediaFormat.KEY_COLOR_FORMAT, MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface);
        mediaFormat.setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 33);
        mediaFormat.setInteger(MediaFormat.KEY_BIT_RATE, 6000000);
        mediaFormat.setInteger(MediaFormat.KEY_CAPTURE_RATE, 30);

        try {
            outChannel = new FileOutputStream(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_MOVIES).getAbsolutePath() + "/capture.avc").getChannel();
            mediaCodec = MediaCodec.createEncoderByType(MediaFormat.MIMETYPE_VIDEO_AVC);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

        mediaCodec.configure(mediaFormat, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE);
        outSurface = mediaCodec.createInputSurface();

        mediaCodec.setCallback(new MediaCodec.Callback() {
            @Override
            public void onInputBufferAvailable(@NonNull MediaCodec codec, int index) {

            }

            @Override
            public void onOutputBufferAvailable(@NonNull MediaCodec codec, int index, @NonNull MediaCodec.BufferInfo info) {
                ByteBuffer outputBuffer = mediaCodec.getOutputBuffer(index);
                try {
                    outChannel.write(outputBuffer);
                } catch (IOException e) {
                    e.printStackTrace();
                }

                mediaCodec.releaseOutputBuffer(index, false);
            }

            @Override
            public void onError(@NonNull MediaCodec codec, @NonNull MediaCodec.CodecException e) {

            }

            @Override
            public void onOutputFormatChanged(@NonNull MediaCodec codec, @NonNull MediaFormat format) {

            }
        });
        mediaCodec.start();
    }



    /**
     * This method shall be called by the subclasses when they have valid
     * object and want it to be delivered to external client (via callback) and
     * then displayed on the screen.
     */
    protected void deliverAndDrawFrame(Mat modified, Matrix transformMat) {
        boolean bmpValid = true;
        if (modified != null) {
            try {
                Utils.matToBitmap(modified, mCacheBitmap);
            } catch(Exception e) {
                Log.e(TAG, "Mat type: " + modified);
                Log.e(TAG, "Bitmap type: " + mCacheBitmap.getWidth() + "*" + mCacheBitmap.getHeight());
                Log.e(TAG, "Utils.matToBitmap() throws an exception: " + e.getMessage());
                bmpValid = false;
            }
        }

        if (bmpValid && mCacheBitmap != null) {
            Canvas canvas = getHolder().lockCanvas();
            if (canvas != null) {
                canvas.drawColor(0, android.graphics.PorterDuff.Mode.CLEAR);

                canvas.rotate(90, mFrameHeight / 2 * mScale, mFrameHeight / 2 * mScale);
                canvas.translate(mFrameOffsetX, -mFrameOffsetY);
                canvas.scale(mScale, mScale);

                canvas.drawBitmap(mCacheBitmap, transformMat, null);

//                if (mFpsMeter != null) {
//                    mFpsMeter.measure();
//                    mFpsMeter.draw(canvas, 20, 30);
//                }
                getHolder().unlockCanvasAndPost(canvas);
            }
        }
        if (outSurface != null) {
            Canvas storageCanvas;
            if (Build.VERSION.SDK_INT >= 23)
                storageCanvas = outSurface.lockHardwareCanvas();
            else
                storageCanvas = outSurface.lockCanvas(surfaceRect);
            if (storageCanvas != null) {
                storageCanvas.drawColor(0, android.graphics.PorterDuff.Mode.CLEAR);
                if (BuildConfig.DEBUG)
                    Log.d(TAG, "mStretch value: " + mScale);

                if (transformMat == null) {
                    transformMat = new Matrix();
                }

                // 修正角度
                storageCanvas.rotate(90, mFrameHeight / 2 * 1.25f, mFrameHeight / 2 * 1.25f);
                storageCanvas.translate(-160, 90);
                storageCanvas.scale(1.25f, 1.25f); // TODO crop ration 0.8

                storageCanvas.drawBitmap(mCacheBitmap, transformMat, null);

                outSurface.unlockCanvasAndPost(storageCanvas);
            }
        }
    }


    public void startDisplayThread() {
        new Thread(new DisplayThread()).start();
    }

    public class DisplayThread implements Runnable {
        Mat transVec = new Mat(3, 3, CvType.CV_64F);
        Mat outputMat = new Mat(mFrameHeight, mFrameWidth, CvType.CV_8UC3);
        Mat rsMat = new Mat(30, 3, CvType.CV_64F);
        double[] transData = new double[9];
        float[] transDataF = new float[9];

        @Override
        public void run() {
            Thread.currentThread().setName("Display Thread");
//            long lastFrameTime = SystemClock.uptimeMillis();
//            int turns = 0;
//            initCodec();
            while (true) {
                synchronized (syncObj) {
                    try {
                        syncObj.wait(30);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                if (stableProcessor != null) {
                    stableProcessor.dequeueOutputBuffer(transVec, outputMat, rsMat);
                    Log.e(TAG, "rsMat-------------"+rsMat.dump());
                }

                if (outputMat.empty()) {
                    Log.i(TAG, "yes, get empty output Mat");
                    break;
                }
                if (transVec.type() != CvType.CV_64F) {
                    Log.e(TAG, "WTF?? " + transVec.dump() + outputMat.dump());
                    Log.e(TAG, transVec + " " + outputMat);
                }
                transVec.get(0, 0, transData);
                for (int i = 0; i < 9; i++) {
                    transDataF[i] = (float) transData[i];

                }
                Log.e(TAG,"transDataF: "+transDataF[0]);
                double o00[]=outputMat.get(0,0);
                Log.e(TAG,"outputMat: "+o00[0]);

//                for (int i = 0; i < 6; i ++) {
//                    transDataF[i] = (float)transData[i];
//                }
//                transDataF[6] = 0;
//                transDataF[7] = 0;
//                transDataF[8] = 1;
//                transMatrix.setValues(transDataF);

//                renderer.transVec = transVec;
                renderer.transformMat = transDataF;
//                outputMat.put(100, 1900, 0);
                renderer.outputMat = outputMat;
                renderer.rsMat = rsMat;
                if (renderer.isReady) {
                    CameraBridgeViewBase.this.requestRender();
                }
//                deliverAndDrawFrame(outputMat, transMatrix);

                if (stableProcessor != null) {
                    stableProcessor.enqueueOutputBuffer();
                }
            }

//                try {
//                    while(SystemClock.uptimeMillis() - lastFrameTime < 1000 / mFpsMeter.getFps() - 5) {
//                        Thread.sleep(2);
//                    }
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                } finally {
//                    lastFrameTime = SystemClock.uptimeMillis();
//                }
//                Log.i(TAG, "Put image back");
        }
    }


}
