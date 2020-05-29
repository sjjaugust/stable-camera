package me.zhehua.gryostable.filter;

import android.content.Context;
import android.opengl.GLES30;
import android.util.Log;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
import java.util.Arrays;

import me.zhehua.gryostable.util.OpenGlUtils;
import me.zhehua.gryostable.widget.GlRenderView;

public class BaseFilter {
    private String TAG = "BaseFilter";
    private Context mContext;
    protected int mVertexShaderId ;
    protected int mFragShaderId;
    protected final FloatBuffer mGlVertexBuffer;
    protected final FloatBuffer mGlTextureBuffer;
    protected final ShortBuffer mGlIndexBuffer;
    protected String mVertexShader;
    protected String mFragShader;
    protected int mProgramId;
    protected int SamplerY;
    protected int SamplerU;
    protected int transform;
    protected int vPosition;
    protected int inTexCoord;
    protected int mOutputHeight;
    protected int mOutputWidth;
    protected int y;
    protected int x;
    short[] indices = {
            0, 1, 3,
            3, 2, 0,
            2, 3, 5,
            5, 4, 2,
            4, 5, 7,
            7, 6, 4,
            6, 7, 9,
            9, 8, 6,
            8, 9, 11,
            11, 10, 8,
            10, 11, 13,
            13, 12, 10,
            12, 13, 15,
            15, 14, 12,
            14, 15, 17,
            17, 16, 14,
            16, 17, 19,
            19, 18, 16,
            18, 19, 21,
            21, 20, 18
    };
    float[] VERTEX = {
            1f, 1f, 0f,
            1f, -1f, 0f,
            0.8f, 1f, 0f,
            0.8f, -1f, 0f,
            0.6f, 1f, 0f,
            0.6f, -1f, 0f,
            0.4f, 1f, 0f,
            0.4f, -1f, 0f,
            0.2f, 1f, 0f,
            0.2f, -1f, 0f,
            0f, 1f, 0f,
            0f, -1f, 0f,
            -0.2f, 1f, 0f,
            -0.2f, -1f, 0f,
            -0.4f, 1f, 0f,
            -0.4f, -1f, 0f,
            -0.6f, 1f, 0f,
            -0.6f, -1f, 0f,
            -0.8f, 1f, 0f,
            -0.8f, -1f, 0f,
            -1f, 1f, 0f,
            -1f, -1f, 0f
    };
    float[] VERTEX1 = {
            1f, 1f, 0f,
            1f, -1f, 0f,
            0.8f, 1f, 0f,
            0.8f, -1f, 0f,
            0.6f, 1f, 0f,
            0.6f, -1f, 0f,
            0.4f, 1f, 0f,
            0.4f, -1f, 0f,
            0.2f, 1f, 0f,
            0.2f, -1f, 0f,
            0f, 1f, 0f,
            0f, -1f, 0f,
            -0.2f, 1f, 0f,
            -0.2f, -1f, 0f,
            -0.4f, 1f, 0f,
            -0.4f, -1f, 0f,
            -0.6f, 1f, 0f,
            -0.6f, -1f, 0f,
            -0.8f, 1f, 0f,
            -0.8f, -1f, 0f,
            -1f, 1f, 0f,
            -1f, -1f, 0f
    };
    public float[] transformMatrix;
    public Mat outputMat;
    public ByteBuffer yPlane;
    public ByteBuffer uPlane;
    byte[] yBytes;
    byte[] uBytes;
    public int videoWidth;
    public int videoHeight;
    protected boolean isOpenRollingShutter = true;
    public Mat rsMat;

    public BaseFilter(Context mContext, int mVertexShaderId, int mFragShaderId) {
        this.mContext = mContext;
        this.mVertexShaderId = mVertexShaderId;
        this.mFragShaderId = mFragShaderId;

        mGlVertexBuffer = ByteBuffer.allocateDirect(VERTEX.length* 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
        mGlVertexBuffer.clear();
//        mGlVertexBuffer.put(VERTEX);


        float[] TEXTURE = {
                0f, 0f,
                1f, 0f,
                0f, 0.1f,
                1f, 0.1f,
                0f, 0.2f,
                1f, 0.2f,
                0f, 0.3f,
                1f, 0.3f,
                0f, 0.4f,
                1f, 0.4f,
                0f, 0.5f,
                1f, 0.5f,
                0f, 0.6f,
                1f, 0.6f,
                0f, 0.7f,
                1f, 0.7f,
                0f, 0.8f,
                1f, 0.8f,
                0f, 0.9f,
                1f, 0.9f,
                0f, 1f,
                1f, 1f
        };
        mGlTextureBuffer = ByteBuffer.allocateDirect(TEXTURE.length * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
        mGlTextureBuffer.clear();
        mGlTextureBuffer.put(TEXTURE);

        mGlIndexBuffer = ByteBuffer.allocateDirect(indices.length*2).order(ByteOrder.nativeOrder()).asShortBuffer();
        mGlIndexBuffer.clear();
        mGlIndexBuffer.put(indices);

        initilize(mContext);
        resetCoordinate();
        transformMatrix = new float[] {
                1.0f, 0f, 0f,
                0f, 1f, 0f,
                0f, 0f, 1f
        };
        rsMat = new Mat(30, 3, CvType.CV_64F);
        for(int i = 0; i < 30; i++){
            if(i%3 == 0){
                rsMat.get(i, 0)[0] = 1.0f;
                rsMat.get(i, 1)[0] = 0.0f;
                rsMat.get(i, 2)[0] = 0.0f;
            }
            else if (i%3 == 1){
                rsMat.get(i, 0)[0] = 0.0f;
                rsMat.get(i, 1)[0] = 1.0f;
                rsMat.get(i, 2)[0] = 0.0f;
            }
            else if (i%3 == 2){
                rsMat.get(i, 0)[0] = 0.0f;
                rsMat.get(i, 1)[0] = 0.0f;
                rsMat.get(i, 2)[0] = 1.0f;
            }
        }
    }
    private void initilize(Context mContext){
        mVertexShader = OpenGlUtils.readRawShaderFile(mContext, mVertexShaderId);
        mFragShader = OpenGlUtils.readRawShaderFile(mContext, mFragShaderId);
        mProgramId = OpenGlUtils.loadProgram(mVertexShader, mFragShader);
        vPosition = GLES30.glGetAttribLocation(mProgramId, "vPosition");
        inTexCoord = GLES30.glGetAttribLocation(mProgramId, "inTexCoord");
        transform = GLES30.glGetUniformLocation(mProgramId, "transform");
        SamplerY = GLES30.glGetUniformLocation(mProgramId, "SamplerY");
        SamplerU = GLES30.glGetUniformLocation(mProgramId, "SamplerU");
    }

    public void prepare(int width, int height, int x, int y) {
        mOutputWidth = width;
        mOutputHeight = height;
        this.x = x;
        this.y = y;

    }

    public int[] onDrawFrame(int[] textureId){
        if(outputMat != null){

            videoHeight = outputMat.rows() / 3 * 2;
            videoWidth = outputMat.cols();
            if (yPlane == null || yBytes == null) {
                yPlane = ByteBuffer.allocateDirect(videoHeight * videoWidth);
                uPlane = ByteBuffer.allocateDirect(videoHeight * videoWidth / 2);
                yBytes = new byte[videoWidth * videoHeight];
                uBytes = new byte[videoWidth * videoHeight / 2];
            }
            yPlane.clear();
            uPlane.clear();
            outputMat.get(0, 0, yBytes);
            outputMat.get(videoHeight, 0, uBytes);
            yPlane.put(yBytes);
            uPlane.put(uBytes);
            yPlane.position(0);
            uPlane.position(0);
        }


//

//        GLES30.glViewport(x, y, mOutputWidth, mOutputHeight);
        GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT | GLES30.GL_DEPTH_BUFFER_BIT);
        GLES30.glUseProgram(mProgramId);
        //传入顶点
        if(isOpenRollingShutter){
            changeVertex(rsMat);
        }
        mGlVertexBuffer.put(VERTEX);
        mGlVertexBuffer.position(0);
        GLES30.glVertexAttribPointer(vPosition, 3, GLES30.GL_FLOAT, false, 3*4, mGlVertexBuffer);
        GLES30.glEnableVertexAttribArray(vPosition);
        Log.d(TAG, "onDrawFramebuffer: "+mGlTextureBuffer.get(43));
        //传入纹理坐标
        mGlTextureBuffer.position(0);
        GLES30.glVertexAttribPointer(inTexCoord, 4, GLES30.GL_FLOAT, false, 2*4, mGlTextureBuffer);
        GLES30.glEnableVertexAttribArray(inTexCoord);
        //传入变换矩阵
        GLES30.glUniformMatrix3fv(transform, 1, true, transformMatrix, 0);
        //传入纹理

        GLES30.glActiveTexture(GLES30.GL_TEXTURE0);
        GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, textureId[0]);
        GLES30.glTexImage2D(GLES30.GL_TEXTURE_2D, 0, GLES30.GL_LUMINANCE, videoWidth, videoHeight, 0,
                GLES30.GL_LUMINANCE, GLES30.GL_UNSIGNED_BYTE, yPlane);
        GLES30.glUniform1i(SamplerY, 0);
        GLES30.glActiveTexture(GLES30.GL_TEXTURE1);
        GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, textureId[1]);
        GLES30.glTexImage2D(GLES30.GL_TEXTURE_2D, 0, GLES30.GL_LUMINANCE_ALPHA, videoWidth / 2, videoHeight / 2, 0,
                GLES30.GL_LUMINANCE_ALPHA, GLES30.GL_UNSIGNED_BYTE, uPlane); // TODO
        GLES30.glUniform1i(SamplerU, 1);
        //画

        mGlIndexBuffer.position(0);
        GLES30.glDrawElements(GLES30.GL_TRIANGLES, indices.length, GLES30.GL_UNSIGNED_SHORT, mGlIndexBuffer);

        GLES30.glDisableVertexAttribArray(vPosition);
        GLES30.glDisableVertexAttribArray(inTexCoord);
        GLES30.glFlush();

        return textureId;
    }

    public void release(){
        GLES30.glDeleteProgram(mProgramId);
    }

    protected void resetCoordinate() {

    }
    public void setMatrix(float[] transform) {
        this.transformMatrix = transform;
    }

    public void setIsOpenRollingShutter(boolean status){
        isOpenRollingShutter = status;
    }

    protected void changeVertex(Mat rsMat){

        int rsMatPosition = 0;
        int rsMatStride = 3;
        int vertexPosition = 0;
        int vertexStride = 6;
        double[] vertexT1 = new double[3];
        double[] vertexT2 = new double[3];

        for(int i = 0; i < 10; i++){
            vertexT1[0] = VERTEX1[vertexPosition];
            vertexT1[1] = VERTEX1[vertexPosition+1];
            vertexT1[2] = VERTEX1[vertexPosition+2];
            vertexT2[0] = VERTEX1[vertexPosition+3];
            vertexT2[1] = VERTEX1[vertexPosition+4];
            vertexT2[2] = VERTEX1[vertexPosition+5];

            Mat rsMatTemp = rsMat.rowRange(rsMatPosition, rsMatPosition+3);
//            Log.e(TAG, "changeVertex: "+rsMatPosition+rsMatTemp.dump() );
            double[] dst = mul(rsMatTemp, vertexT1);
            for(int j = 0; j < 3; j++){
                if(Math.abs(dst[j])<1e-3){
                    dst[j] = 0;
                }
            }
            VERTEX[vertexPosition] = (float)dst[0];
            VERTEX[vertexPosition+1] = (float)dst[1];
            VERTEX[vertexPosition+2] = (float)dst[2];
            dst = mul(rsMatTemp, vertexT2);
            for(int j = 0; j < 3; j++){
                if(Math.abs(dst[j])<1e-3){
                    dst[j] = 0;
                }
            }
            VERTEX[vertexPosition+3] = (float)dst[0];
            VERTEX[vertexPosition+4] = (float)dst[1];
            VERTEX[vertexPosition+5] = (float)dst[2];
            rsMatPosition += rsMatStride;
            vertexPosition += vertexStride;
        }
        //计算最后一条
        double[] vertexLast1 = new double[3];
        double[] vertexLast2 = new double[3];
        int tempPosition = vertexPosition;
        for(int i = 0; i < 3; i++){
            vertexLast1[i] = VERTEX1[vertexPosition++];
        }
        for(int i = 0; i < 3; i++){
            vertexLast2[i] = VERTEX1[vertexPosition++];
        }
        Mat rsMatTemp = rsMat.rowRange(rsMatPosition-3, rsMatPosition);
        double[] dst = mul(rsMatTemp, vertexLast1);
        for(int j = 0; j < 3; j++){
            if(Math.abs(dst[j])<1e-3){
                dst[j] = 0;
            }
        }
        VERTEX[tempPosition] = (float) dst[0];
        VERTEX[tempPosition+1] = (float) dst[1];
        VERTEX[tempPosition+2] = (float) dst[2];
        dst = mul(rsMatTemp, vertexLast2);
        for(int j = 0; j < 3; j++){
            if(Math.abs(dst[j])<1e-3){
                dst[j] = 0;
            }
        }
        VERTEX[tempPosition+3] = (float) dst[0];
        VERTEX[tempPosition+4] = (float) dst[1];
        VERTEX[tempPosition+5] = (float) dst[2];

        Log.e(TAG, "changeVertex11111: "+ Arrays.toString(VERTEX));

    }
    private double[] mul(Mat src1, double[] src2){
        double[] dst = new double[3];
        for(int i = 0; i < 3; i++){
            dst[i] = src1.get(i,0)[0]*src2[0]+src1.get(i, 1)[0]*src2[1]+src1.get(i, 2)[0]*src2[2];
        }
        return dst;


    }
}
