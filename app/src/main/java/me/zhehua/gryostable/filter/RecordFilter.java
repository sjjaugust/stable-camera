package me.zhehua.gryostable.filter;

import android.content.Context;
import android.opengl.GLES30;
import android.util.Log;

import me.zhehua.gryostable.R;

public class RecordFilter extends BaseFilter {

    public RecordFilter(Context mContext){
        super(mContext, R.raw.vertex, R.raw.fragment);
    }

    @Override
    public int[] onDrawFrame(int[] textureId) {
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
        //传入纹理坐标
        mGlTextureBuffer.position(0);
        GLES30.glVertexAttribPointer(inTexCoord, 4, GLES30.GL_FLOAT, false, 2*4, mGlTextureBuffer);
        GLES30.glEnableVertexAttribArray(inTexCoord);
        //传入变换矩阵
        GLES30.glUniformMatrix3fv(transform, 1, true, transformMatrix, 0);
        //传入纹理

        GLES30.glActiveTexture(GLES30.GL_TEXTURE0);
        GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, textureId[0]);

        GLES30.glUniform1i(SamplerY, 0);
        GLES30.glActiveTexture(GLES30.GL_TEXTURE1);
        GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, textureId[1]);

        GLES30.glUniform1i(SamplerU, 1);
        //画
        mGlIndexBuffer.position(0);
        GLES30.glDrawElements(GLES30.GL_TRIANGLES, indices.length, GLES30.GL_UNSIGNED_SHORT, mGlIndexBuffer);

        GLES30.glDisableVertexAttribArray(vPosition);
        GLES30.glDisableVertexAttribArray(inTexCoord);
        GLES30.glFlush();

        return textureId;
    }
}
