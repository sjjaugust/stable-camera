package me.zhehua.gryostable.filter;

import android.content.Context;
import android.opengl.GLES30;
import android.util.Log;

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
    public float[] transformMatrix;
    public Mat outputMat;
    public ByteBuffer yPlane;
    public ByteBuffer uPlane;
    byte[] yBytes;
    byte[] uBytes;
    private int videoWidth;
    private int videoHeight;

    public BaseFilter(Context mContext, int mVertexShaderId, int mFragShaderId) {
        this.mContext = mContext;
        this.mVertexShaderId = mVertexShaderId;
        this.mFragShaderId = mFragShaderId;
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
        mGlVertexBuffer = ByteBuffer.allocateDirect(VERTEX.length* 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
        mGlVertexBuffer.clear();
        mGlVertexBuffer.put(VERTEX);


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
            Log.d(TAG, "onDrawFramesize: "+outputMat.get(0, 0)[0]);
            videoHeight = outputMat.rows() / 3 * 2; // TODO
            videoWidth = outputMat.cols();
            if (yPlane == null || yBytes == null) {
                yPlane = ByteBuffer.allocateDirect(videoHeight * videoWidth);
                uPlane = ByteBuffer.allocateDirect(videoHeight * videoWidth / 2); // TODO
                yBytes = new byte[videoWidth * videoHeight];
                uBytes = new byte[videoWidth * videoHeight / 2]; // TODO
            }
            yPlane.clear();
            uPlane.clear();
            outputMat.get(0, 0, yBytes);
            outputMat.get(videoHeight, 0, uBytes);
            Log.d(TAG, "onDrawFrame11111: "+ yBytes[0]);
        }

//        GLES30.glViewport(x, y, mOutputWidth, mOutputHeight);
        GLES30.glUseProgram(mProgramId);
        //传入顶点
        mGlVertexBuffer.position(0);
        GLES30.glVertexAttribPointer(vPosition, 3, GLES30.GL_FLOAT, false, 3*4, mGlVertexBuffer);
        GLES30.glEnableVertexAttribArray(vPosition);
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
}
