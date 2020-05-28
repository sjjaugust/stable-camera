package me.zhehua.gryostable.filter;

import android.content.Context;
import android.opengl.GLES30;

import me.zhehua.gryostable.util.OpenGlUtils;

public class AbstractFBOFilter extends BaseFilter {

    protected int[] mFrameBuffers;
    protected int[] mFBOTextures;

    public AbstractFBOFilter(Context mContext, int mVertexShaderId, int mFragShaderId) {
        super(mContext, mVertexShaderId, mFragShaderId);
    }

    @Override
    public void prepare(int width, int height,int x,int y) {
        super.prepare(width, height,x,y);

        loadFBO();

    }

    private void loadFBO(){
        if(mFrameBuffers != null){
            destroyFrameBuffers();
        }
        mFrameBuffers = new int[1];
        GLES30.glGenFramebuffers(mFrameBuffers.length, mFrameBuffers, 0);
        //穿件FBO中的纹理
        mFBOTextures = new int[2];
        OpenGlUtils.glGenTextures(mFBOTextures);

        GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, mFBOTextures[0]);
        //指定FBO纹理的输出图像的格式 RGBA
        GLES30.glTexImage2D(GLES30.GL_TEXTURE_2D, 0, GLES30.GL_RGBA, mOutputWidth, mOutputHeight,
        0, GLES30.GL_RGBA, GLES30.GL_UNSIGNED_BYTE, null);

        GLES30.glBindFramebuffer(GLES30.GL_FRAMEBUFFER, mFrameBuffers[0]);

        //将fbo绑定到2d的纹理上
        GLES30.glFramebufferTexture2D(GLES30.GL_FRAMEBUFFER, GLES30.GL_COLOR_ATTACHMENT0,
                GLES30.GL_TEXTURE_2D, mFBOTextures[0], 0);

        GLES30.glBindTexture(GLES30.GL_TEXTURE_2D, 0);
        GLES30.glBindFramebuffer(GLES30.GL_FRAMEBUFFER, 0);
    }

    public void destroyFrameBuffers() {
        //删除fbo的纹理
        if (mFBOTextures != null) {
            GLES30.glDeleteTextures(1, mFBOTextures, 0);
            mFBOTextures = null;
        }
        //删除fbo
        if (mFrameBuffers != null) {
            GLES30.glDeleteFramebuffers(1, mFrameBuffers, 0);
            mFrameBuffers = null;
        }
    }

}
