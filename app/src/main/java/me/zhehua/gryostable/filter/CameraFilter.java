package me.zhehua.gryostable.filter;

import android.content.Context;
import android.opengl.GLES11Ext;
import android.opengl.GLES30;

import me.zhehua.gryostable.R;

public class CameraFilter extends AbstractFBOFilter {


    public CameraFilter(Context mContext) {
        super(mContext, R.raw.vertex, R.raw.fragment);
    }


    @Override
    public int[] onDrawFrame(int[] textureId) {
        GLES30.glViewport(0, 0, mOutputWidth, mOutputHeight);
        GLES30.glBindFramebuffer(GLES30.GL_FRAMEBUFFER, mFrameBuffers[0]);
        GLES30.glUseProgram(mProgramId);
        //传入顶点坐标
        mGlVertexBuffer.position(0);
        GLES30.glVertexAttribPointer(vPosition, 3, GLES30.GL_FLOAT, false, 3*4, mGlVertexBuffer);
        GLES30.glEnableVertexAttribArray(vPosition);
        //传入纹理坐标
        mGlTextureBuffer.position(0);
        GLES30.glVertexAttribPointer(inTexCoord, 2, GLES30.GL_FLOAT, false, 2*4, mGlTextureBuffer);
        GLES30.glEnableVertexAttribArray(inTexCoord);
        //传入转换矩阵
        GLES30.glUniformMatrix3fv(transform, 1, false, transformMatrix, 0);
        //传入纹理
        GLES30.glActiveTexture(GLES30.GL_TEXTURE0);
        GLES30.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, textureId[0]);
        GLES30.glUniform1i(SamplerY, 0);
        GLES30.glActiveTexture(GLES30.GL_TEXTURE1);
        GLES30.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, textureId[1]);
        GLES30.glUniform1i(SamplerU, 1);
        //画
        GLES30.glDrawElements(GLES30.GL_TRIANGLES, indices.length, GLES30.GL_UNSIGNED_SHORT, mGlTextureBuffer);
        //解除绑定
        GLES30.glBindFramebuffer(GLES30.GL_FRAMEBUFFER, 0);
        GLES30.glBindTexture(GLES11Ext.GL_TEXTURE_EXTERNAL_OES, 0);

        return mFBOTextures;
    }
}
