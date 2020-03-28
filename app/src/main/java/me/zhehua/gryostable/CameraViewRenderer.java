package me.zhehua.gryostable;

import android.content.Context;
import android.content.res.AssetManager;
import android.opengl.GLES30;
import android.opengl.GLSurfaceView;
import android.util.Log;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Range;
import org.opencv.core.Size;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.math.BigDecimal;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
import java.util.Arrays;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import me.zhehua.gryostable.utils.ArithUtil;

import static org.opencv.core.Mat.zeros;

/**
 * Created by zhehua on 05/11/2017.
 */

public class CameraViewRenderer implements GLSurfaceView.Renderer {
//    Mat transVec;
    Mat outputMat;

    protected CameraBridgeViewBase mView;

    private final static String TAG = "MyRenderer";
    private Context context;
    private FloatBuffer vertexBuffer;
    private ShortBuffer indexBuffer;
    private FloatBuffer texCoordBuffer;
    private int program;
    private int positionHandle;
    private int texPosHandle;
    private int texSamplerYHandle;
    private int transformHandle;
    private int texSamplerUHandle;
    private int texSamplerVHandle;
    private int[] texName;
    public float[] transformMat;
    public ByteBuffer yPlane;
    public ByteBuffer uPlane;
    public ByteBuffer vPlane;
    byte[] yBytes;
    byte[] uBytes;
    byte[] vBytes;
    public boolean isReady = false;
    public volatile boolean isFree = true;
    private int videoWidth;
    private int videoHeight;
    public final Object syncObj = new Object();
    //果冻效应
    private int rsMatHandle;
    Mat rsMat;
//    private static int rsStripNum = 10;
//    private static int VERTEXSIZE = 3*2*(rsStripNum+1);
//    private static int TEXTURESIZE = 2*2*(rsStripNum+1);
//    private static int INDICESIZE = (2*(rsStripNum+1)-2)*3;

    private float[] vertex = {
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
    private float[] vertex1 = {
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
    private float[] texCoor = {
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
    private short[] indices = {
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

//    private static float[] vertex = new float[VERTEXSIZE];
//    private static short[] indices = new short[INDICESIZE];
//    private static float[] texCoor = new float[TEXTURESIZE];
//
//    private void setArray(){

        //顶点数组
//        float strip = 2f/rsStripNum;
//        int stride = 6;
//        float x = -1f;
//        float y = 1f;
//        float z = 0;
//        for(int i = 0; i < VERTEXSIZE; i+=stride){
//            vertex[i] = x;
//            vertex[i+1] = y;
//            vertex[i+2] = z;
//            x = -x;
//            vertex[i+3] = x;
//            vertex[i+4] = y;
//            vertex[i+5] = z;
//            x = -x;
//            y -= strip;
//        }

//        int k = 0;
//        for(int i = 0; i < INDICESIZE; i+=stride){
//            indices[i] = (short) k;
//            indices[i+1] = (short)(k+1);
//            indices[i+2] = (short)(k+3);
//            indices[i+3] = (short)(k+3);
//            indices[i+4] = (short)(k+2);
//            indices[i+5] = (short)k;
//            k+=2;
//
//        }
//        Log.e(TAG, "vertex:i am in" );
//
//        stride = 4;
//        x = 0f;
//        y = 0f;
//        strip = 1f/rsStripNum;
//        Log.e(TAG, "wtf-----1strip"+strip );
//        for(int i = 0 ; i < TEXTURESIZE; i += stride){
//            texCoor[i] = x;
//            texCoor[i+1] = y==1f?0:1f;
//            y = texCoor[i+1];
//            texCoor[i+2] = x;
//            texCoor[i+3] = y==1f?0:1f;
//            y = texCoor[i+3];
//            x+=strip;
//        }
//
//        Log.e(TAG, "wtf-----1vertex:"+ Arrays.toString(vertex));
//        Log.e(TAG, "wtf-----1indices:"+Arrays.toString(indices));
//        Log.e(TAG, "wtf-----1texturecoord:"+Arrays.toString(texCoor));
//    }

//原来
//    private static float[] vertex = {
//            -1f, 1f, 0, // left top
//            1f, 1f, 0, // right top
//            -1f, -1f, 0, // left bottom
//            1f, -1f, 0,// right bottom
//            -1f, 0, 0,
//            1f, 0, 0
//    };
//
//    private static short[] indices = {
//            0, 1, 4,
//            1, 4, 5,
//            4, 5, 2,
//            5, 2, 3
//    };

    // hdmi
//    private static float[] texCoor = {
//            0, 0,
//            1, 0,
//            0, 1,
//            1, 1
//    };

    // phone
//    private static float[] texCoor = {
//            0.1f, 0.9f,
//            0.1f, 0.1f,
//            0.9f, 0.9f,
//            0.9f, 0.1f
//    };
//原来
//    private static float[] texCoor = {
//            0.0f, 1.0f,
//            0.0f, 0.0f,
//            1.0f, 1.0f,
//            1.0f, 0.0f,
//            0.5f, 1.0f,
//            0.5f, 0.0f
//
//
//    };

    //mirror
//    private static float[] texCoor = {
//            0.0f, 0.0f,
//            0.0f, 1.0f,
//            1.0f, 0.0f,
//            1.0f, 1.0f
//    };


    public CameraViewRenderer(Context context) {
        this.context = context;
        this.texName = new int[3];
//        vertexBuffer = ByteBuffer.allocateDirect(vertex.length * 4)
//                .order(ByteOrder.nativeOrder())
//                .asFloatBuffer()
//                .put(vertex);
//        vertexBuffer.position(0);

        indexBuffer = ByteBuffer.allocateDirect(indices.length * 2)
                .order(ByteOrder.nativeOrder())
                .asShortBuffer()
                .put(indices);
        indexBuffer.position(0);

        texCoordBuffer = ByteBuffer.allocateDirect(texCoor.length * 4)
                .order(ByteOrder.nativeOrder())
                .asFloatBuffer()
                .put(texCoor);
        texCoordBuffer.position(0);

//        transformMat = new float[] {
//                1.732f/2, -1f/2f, 0.0f,
//                1f/2f, 1.732f/2, 0.0f,
//                0.0f, 0.0f, 1.0f
//        };

        transformMat = new float[] {
                1.0f, 0f, 0f,
                0f, 1f, 0f,
                0f, 0f, 1f
        };
//        rsMat = new float[]{
//            1.0f, 0f, 0f,
//            0f, 1f, 0f,
//            0f, 0f, 1f
//        };


    }
    @Override
    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
//        setArray();


    }

    private String readAssets(String filename) throws IOException {
        StringBuilder sb = new StringBuilder();
        InputStream is = context.getAssets().open(filename);
        BufferedReader br = new BufferedReader(new InputStreamReader(is, "UTF-8"));
        String str;
        while ((str = br.readLine()) != null) {
            sb.append(str + '\n');
        }
        br.close();
        return sb.toString();
    }

    @Override
    public void onSurfaceChanged(GL10 gl, int width, int height) {

        program = GLES30.glCreateProgram();
        int vertexShader = GLES30.glCreateShader(GLES30.GL_VERTEX_SHADER);
        try {
            String vertexShaderSource = readAssets("shader.glslv");
            GLES30.glShaderSource(vertexShader, vertexShaderSource);
            GLES30.glCompileShader(vertexShader);
//            Log.e(TAG, "vert" + GLES30.glGetShaderInfoLog(vertexShader));
        } catch (IOException e) {
            e.printStackTrace();
        }
        int fragmentShader = GLES30.glCreateShader(GLES30.GL_FRAGMENT_SHADER);
        try {
            String fragmentShaderSource = readAssets("shader.glslf");
            GLES30.glShaderSource(fragmentShader, fragmentShaderSource);
            GLES30.glCompileShader(fragmentShader);
        } catch (IOException e) {
            e.printStackTrace();
        }

//        Log.e(TAG, "frag" + GLES30.glGetShaderInfoLog(fragmentShader));

        GLES30.glAttachShader(program, vertexShader);
        GLES30.glAttachShader(program, fragmentShader);
        GLES30.glLinkProgram(program);

        positionHandle = GLES30.glGetAttribLocation(program, "vPosition");
        texPosHandle = GLES30.glGetAttribLocation(program, "inTexCoord");
        texSamplerYHandle = GLES30.glGetUniformLocation(program, "SamplerY");
        texSamplerUHandle = GLES30.glGetUniformLocation(program, "SamplerU");
        texSamplerVHandle = GLES30.glGetUniformLocation(program, "SamplerV");
        transformHandle = GLES30.glGetUniformLocation(program, "transform");
        //果冻效应
//        rsMatHandle = GLES30.glGetUniformLocation(program, "rsMat");
//        GLES30.glVertexAttribPointer(positionHandle, 3, GLES30.GL_FLOAT, false, 3 * 4, vertexBuffer);
        GLES30.glVertexAttribPointer(texPosHandle, 2, GLES30.GL_FLOAT, false, 2 * 4, texCoordBuffer);

        GLES30.glGenTextures(3, texName, 0);
        GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, texName[0]);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MIN_FILTER,
                GLES30.GL_LINEAR);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MAG_FILTER,
                GLES30.GL_LINEAR);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_S,
                GLES30.GL_REPEAT);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_T,
                GLES30.GL_REPEAT);
        GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, texName[1]);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MIN_FILTER,
                GLES30.GL_LINEAR);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MAG_FILTER,
                GLES30.GL_LINEAR);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_S,
                GLES30.GL_REPEAT);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_T,
                GLES30.GL_REPEAT);
        GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, texName[2]);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MIN_FILTER,
                GLES30.GL_LINEAR);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MAG_FILTER,
                GLES30.GL_LINEAR);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_S,
                GLES30.GL_REPEAT);
        GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_T,
                GLES30.GL_REPEAT);


        isReady = true;
    }

    @Override
    public void onDrawFrame(GL10 gl) {
        isFree = false;

        if (isReady && outputMat != null) {
            videoHeight = outputMat.rows() / 3 * 2; // TODO
            videoWidth = outputMat.cols();

            if (yPlane == null || yBytes == null) {
                yPlane = ByteBuffer.allocateDirect(videoHeight * videoWidth);
                uPlane = ByteBuffer.allocateDirect(videoHeight * videoWidth / 2); // TODO
//                vPlane = ByteBuffer.allocateDirect(videoHeight * videoWidth / 2); // TODO
                yBytes = new byte[videoWidth * videoHeight];
                uBytes = new byte[videoWidth * videoHeight / 2]; // TODO
//                vBytes = new byte[videoWidth * videoHeight / 2]; // TODO
            }
            yPlane.clear();
            uPlane.clear();
//            vPlane.clear();
//            outputMat.get(0, 0, bytes);
            outputMat.get(0, 0, yBytes);
            outputMat.get(videoHeight, 0, uBytes);
            GLES30.glUniformMatrix3fv(transformHandle, 1, true, transformMat, 0);
            //果冻效应
//            float[] dst = convertMatToFloat(rsMat);
//            Log.e(TAG, "dst1111"+rsMat.dump());
//            Log.e(TAG, "dst1111"+Arrays.toString(dst));
//            GLES30.glUniformMatrix3fv(rsMatHandle, 1, false, dst, 0);
            changeVertex(rsMat);
            vertexBuffer = ByteBuffer.allocateDirect(vertex.length * 4)
                    .order(ByteOrder.nativeOrder())
                    .asFloatBuffer()
                    .put(vertex);
            vertexBuffer.position(0);
            GLES30.glVertexAttribPointer(positionHandle, 3, GLES30.GL_FLOAT, false, 3 * 4, vertexBuffer);


//            outputMat.get(videoHeight / 2 * 3, 0, vBytes); // TODO

            yPlane.put(yBytes);
            uPlane.put(uBytes);
//            vPlane.put(vBytes);

            yPlane.position(0);
            uPlane.position(0);
//            vPlane.position(0);

            GLES30.glClear(GLES30.GL_COLOR_BUFFER_BIT | GLES30.GL_DEPTH_BUFFER_BIT);
            GLES30.glUseProgram(program);
            GLES30.glUniform1i(texSamplerYHandle, 0);
            GLES30.glUniform1i(texSamplerUHandle, 1);
            GLES30.glUniform1i(texSamplerVHandle, 2);
            GLES30.glEnableVertexAttribArray(positionHandle);
            GLES30.glEnableVertexAttribArray(texPosHandle);

            GLES30.glActiveTexture(GLES30.GL_TEXTURE0);
//            GLES30.glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, texName[0]);
            GLES30.glTexImage2D(GLES30.GL_TEXTURE_2D, 0, GLES30.GL_LUMINANCE, videoWidth, videoHeight, 0,
                    GLES30.GL_LUMINANCE, GLES30.GL_UNSIGNED_BYTE, yPlane);
            GLES30.glActiveTexture(GLES30.GL_TEXTURE1);
//            GLES30.glPixelStorei(GL_UNPACK_ROW_LENGTH, 1);
            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, texName[1]);
            GLES30.glTexImage2D(GLES30.GL_TEXTURE_2D, 0, GLES30.GL_LUMINANCE_ALPHA, videoWidth / 2, videoHeight / 2, 0,
                    GLES30.GL_LUMINANCE_ALPHA, GLES30.GL_UNSIGNED_BYTE, uPlane); // TODO
//            GLES30.glActiveTexture(GLES30.GL_TEXTURE2);
//            GLES30.glPixelStorei(GL_UNPACK_ROW_LENGTH, 1);
//            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, texName[2]);
//            GLES30.glTexImage2D(GLES30.GL_TEXTURE_2D, 0, GLES30.GL_LUMINANCE, videoWidth, videoHeight / 2, 0,
//                    GLES30.GL_LUMINANCE, GLES30.GL_UNSIGNED_BYTE, uPlane); // TODO
//            Log.i(TAG, transVec.dump());
            GLES30.glDrawElements(GLES30.GL_TRIANGLES, indices.length, GLES30.GL_UNSIGNED_SHORT, indexBuffer);

            GLES30.glDisableVertexAttribArray(positionHandle);
            GLES30.glDisableVertexAttribArray(texPosHandle);
            GLES30.glFlush();
        }
        isFree = true;
    }
    private float[] convertMatToFloat(Mat src){
        int width = (int)src.size().width;
        int height = (int)src.size().height;
        int num = width*height;
        float[] dst = new float[num];
        int position = 0;
        for(int i = 0; i < height; i++){
            for(int j = 0; j < width; j++){
                dst[position] = (float)src.get(i, j)[0];
                position++;
            }
        }
//        Log.e(TAG, "num----"+Arrays.toString(dst));
        return dst;

    }
    private void changeVertex(Mat rsMat){

        int rsMatPosition = 0;
        int rsMatStride = 3;
        int vertexPosition = 0;
        int vertexStride = 6;
        double[] vertexT1 = new double[3];
        double[] vertexT2 = new double[3];

        for(int i = 0; i < 10; i++){
            vertexT1[0] = vertex1[vertexPosition];
            vertexT1[1] = vertex1[vertexPosition+1];
            vertexT1[2] = vertex1[vertexPosition+2];
            vertexT2[0] = vertex1[vertexPosition+3];
            vertexT2[1] = vertex1[vertexPosition+4];
            vertexT2[2] = vertex1[vertexPosition+5];

            Mat rsMatTemp = rsMat.rowRange(rsMatPosition, rsMatPosition+3);
//            Log.e(TAG, "changeVertex: "+rsMatPosition+rsMatTemp.dump() );
            double[] dst = mul(rsMatTemp, vertexT1);
            for(int j = 0; j < 3; j++){
                if(Math.abs(dst[j])<1e-3){
                    dst[j] = 0;
                }
            }
            vertex[vertexPosition] = (float)dst[0];
            vertex[vertexPosition+1] = (float)dst[1];
            vertex[vertexPosition+2] = (float)dst[2];
            dst = mul(rsMatTemp, vertexT2);
            for(int j = 0; j < 3; j++){
                if(Math.abs(dst[j])<1e-3){
                    dst[j] = 0;
                }
            }
            vertex[vertexPosition+3] = (float)dst[0];
            vertex[vertexPosition+4] = (float)dst[1];
            vertex[vertexPosition+5] = (float)dst[2];
            rsMatPosition += rsMatStride;
            vertexPosition += vertexStride;
        }
        //计算最后一条
        double[] vertexLast1 = new double[3];
        double[] vertexLast2 = new double[3];
        int tempPosition = vertexPosition;
        for(int i = 0; i < 3; i++){
            vertexLast1[i] = vertex1[vertexPosition++];
        }
        for(int i = 0; i < 3; i++){
            vertexLast2[i] = vertex1[vertexPosition++];
        }
        Mat rsMatTemp = rsMat.rowRange(rsMatPosition-3, rsMatPosition);
        double[] dst = mul(rsMatTemp, vertexLast1);
        for(int j = 0; j < 3; j++){
            if(Math.abs(dst[j])<1e-3){
                dst[j] = 0;
            }
        }
        vertex[tempPosition] = (float) dst[0];
        vertex[tempPosition+1] = (float) dst[1];
        vertex[tempPosition+2] = (float) dst[2];
        dst = mul(rsMatTemp, vertexLast2);
        for(int j = 0; j < 3; j++){
            if(Math.abs(dst[j])<1e-3){
                dst[j] = 0;
            }
        }
        vertex[tempPosition+3] = (float) dst[0];
        vertex[tempPosition+4] = (float) dst[1];
        vertex[tempPosition+5] = (float) dst[2];

//        Log.e(TAG, "changeVertex11111: "+ Arrays.toString(vertex));

    }
    private double[] mul(Mat src1, double[] src2){
        double[] dst = new double[3];
        for(int i = 0; i < 3; i++){
            dst[i] = src1.get(i,0)[0]*src2[0]+src1.get(i, 1)[0]*src2[1]+src1.get(i, 2)[0]*src2[2];
        }
        return dst;


    }


}
