package me.zhehua.gryostable;

import android.content.Context;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.support.annotation.Nullable;
import android.support.v7.app.AppCompatActivity;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.RadioGroup;

import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

import me.zhehua.gryostable.util.FileUtil;
import me.zhehua.gryostable.util.OnRecordListener;
import me.zhehua.gryostable.widget.GlRenderView;
import me.zhehua.gryostable.widget.RecordButton;

public class MainCameraActivity extends AppCompatActivity implements OnRecordListener {

    private GlRenderView mRenderView;
    static {
        System.loadLibrary("native-lib");
    }


    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.render_view);
        mRenderView = findViewById(R.id.render_view);
        mRenderView.setOnRecordListener(this);

        RecordButton recordButton = findViewById(R.id.btn_record);
        recordButton.setOnRecordListener(new RecordButton.OnRecordListener() {
            @Override
            public void onRecordStart() {
                try{
                    SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMddHHmmss");
                    File file = FileUtil.createFile(MainCameraActivity.this, false, "opengl", sdf.format(new Date(System.currentTimeMillis())) + ".mp4", 1074000000);
                    mRenderView.setSavePath(file.getAbsolutePath());
                    mRenderView.startRecord();
                }catch (FileUtil.NoExternalStoragePermissionException e) {
                    e.printStackTrace();
                } catch (FileUtil.NoExternalStorageMountedException e) {
                    e.printStackTrace();
                } catch (FileUtil.DirHasNoFreeSpaceException e) {
                    e.printStackTrace();
                } catch (IOException e) {
                    e.printStackTrace();
                }

            }

            @Override
            public void onRecordStop() {
                mRenderView.stopRecord();
            }
        });
    }

    @Override
    public void recordFinish(String path) {

    }
}
