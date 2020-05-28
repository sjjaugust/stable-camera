package me.zhehua.gryostable.util;
import android.content.Context;
import android.opengl.GLES20;
import android.opengl.GLES30;
import android.util.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;

public class OpenGlUtils {
    private static String TAG = "OpenGlUtils";
    public static String readRawShaderFile(Context context, int shareId) {
        InputStream is = context.getResources().openRawResource(shareId);
        BufferedReader br = new BufferedReader(new InputStreamReader(is));

        String line;
        StringBuffer sb = new StringBuffer();
        try {

            while ((line = br.readLine()) != null) {
                sb.append(line);
                sb.append("\n");
            }

            br.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
        Log.d(TAG, "readRawShaderFile: " + sb.toString());
        return sb.toString();
    }

    public static int loadProgram(String mVertexShader, String mFragShader) {
        int vshader = GLES30.glCreateShader(GLES30.GL_VERTEX_SHADER);

        GLES30.glShaderSource(vshader, mVertexShader);

        GLES30.glCompileShader(vshader);

        int[] status = new int[1];

        GLES30.glGetShaderiv(vshader, GLES30.GL_COMPILE_STATUS, status, 0);

        if (status[0] != GLES30.GL_TRUE) {
            throw new IllegalStateException("load vertex raw error :" + GLES30.glGetShaderInfoLog(vshader));
        }


        int fshader = GLES30.glCreateShader(GLES30.GL_FRAGMENT_SHADER);

        GLES30.glShaderSource(fshader, mFragShader);

        GLES30.glCompileShader(fshader);


        GLES30.glGetShaderiv(fshader, GLES30.GL_SHADER_COMPILER, status, 0);

        if (status[0] != GLES30.GL_TRUE) {
            throw new IllegalStateException("load fragment raw error :" + GLES30.glGetShaderInfoLog(fshader));
        }


        int programeId = GLES30.glCreateProgram();

        GLES30.glAttachShader(programeId, vshader);
        GLES30.glAttachShader(programeId, fshader);

        GLES30.glLinkProgram(programeId);

        GLES30.glGetProgramiv(programeId, GLES30.GL_LINK_STATUS, status, 0);


        if (status[0] != GLES30.GL_TRUE) {
            throw new IllegalStateException("link program:" + GLES30.glGetProgramInfoLog(programeId));
        }

        GLES30.glDeleteShader(vshader);
        GLES30.glDeleteShader(fshader);

        return programeId;

    }
    public static void glGenTextures(int[] textures) {
        GLES30.glGenTextures(textures.length, textures, 0);


        for (int i = 0; i < textures.length; i++) {
            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, textures[i]);


            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MAG_FILTER, GLES30.GL_NEAREST);
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_MIN_FILTER, GLES30.GL_NEAREST);


            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_S, GLES30.GL_REPEAT);
            GLES30.glTexParameteri(GLES30.GL_TEXTURE_2D, GLES30.GL_TEXTURE_WRAP_T, GLES30.GL_REPEAT);

            GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, 0);

        }
    }
    public static void copyAssets2SdCard(Context context, String src, String dst) {
        try {

            File file = new File(dst);
            if (!file.exists()) {
                InputStream is = context.getAssets().open(src);
                FileOutputStream fos = new FileOutputStream(file);
                int len;
                byte[] buffer = new byte[2048];
                while ((len = is.read(buffer)) != -1) {
                    fos.write(buffer, 0, len);
                }
                is.close();
                fos.close();

            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
