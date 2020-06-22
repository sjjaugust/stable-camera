package me.zhehua.gryostable;

import android.content.Context;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.support.annotation.NonNull;
import android.support.annotation.Nullable;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.RadioGroup;
import android.widget.SeekBar;
import android.widget.TextView;

import java.io.File;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

import me.zhehua.gryostable.util.FileUtil;
import me.zhehua.gryostable.util.OnRecordListener;
import me.zhehua.gryostable.util.PermissionsUtil;
import me.zhehua.gryostable.widget.GlRenderView;
import me.zhehua.gryostable.widget.RecordButton;

public class MainCameraActivity extends AppCompatActivity implements OnRecordListener, PermissionsUtil.IPermissionsCallback{

    private GlRenderView mRenderView;
    static {
        System.loadLibrary("native-lib");
    }
    private String TAG = "MainCameraActivity";
    private TextView textView;
    private Button cropButton;
    private SeekBar seekBar;
    private PermissionsUtil permissionsUtil;
    private Button infoButton;
    private boolean isDraw = true;

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getPermissions();
        setContentView(R.layout.render_view);
        mRenderView = findViewById(R.id.render_view);
        mRenderView.setOnRecordListener(this);
        textView = findViewById(R.id.tv_timedelay);
        textView.setText(String.valueOf((int)(12000000/1000/1000)));
        cropButton = findViewById(R.id.bt_crop);
        cropButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mRenderView.glRender.camera2Helper.isCrop = !mRenderView.glRender.camera2Helper.isCrop;
                if (mRenderView.glRender.camera2Helper.isCrop) {
                    ((Button) v).setText("Crop");
                } else {
                    ((Button) v).setText("Not Crop");
                }
                mRenderView.glRender.camera2Helper.stableProcessor.setCrop(mRenderView.glRender.camera2Helper.isCrop);
            }
        });
        infoButton = findViewById(R.id.bt_draw);
        infoButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                isDraw = !isDraw;
                mRenderView.glRender.camera2Helper.stableProcessor.setDrawStatus(isDraw);
            }
        });
        seekBar = findViewById(R.id.sb_time);
        seekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean b) {
                mRenderView.glRender.camera2Helper.timeDelay = progress * 1000 * 1000;
                textView.setText(String.valueOf((int)(mRenderView.glRender.camera2Helper.timeDelay/1000/1000)));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        RecordButton recordButton = findViewById(R.id.btn_record);
        recordButton.setOnRecordListener(new RecordButton.OnRecordListener() {
            @Override
            public void onRecordStart() {
                try{
                    SimpleDateFormat sdf = new SimpleDateFormat("yyyyMMddHHmmss");

                    File file = FileUtil.createFile(MainCameraActivity.this, false, "opengl", sdf.format(new Date(System.currentTimeMillis())) + ".mp4", 1074000000);
                    mRenderView.setSavePath(file.getAbsolutePath());
                    Log.d(TAG, "onRecordStart: "+file.getAbsolutePath());
                    mRenderView.startRecord();
                    mRenderView.glRender.camera2Helper.stableProcessor.setWriteStatus(true);
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
                mRenderView.glRender.camera2Helper.stableProcessor.setWriteStatus(false);
            }
        });
    }

    @Override
    public void recordFinish(String path) {

    }

    private void getPermissions(){
        permissionsUtil = PermissionsUtil.with(this)
                .requestCode(0)
                .isDebug(true)
                .permissions(PermissionsUtil.Permission.Storage.WRITE_EXTERNAL_STORAGE)
                .request();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        permissionsUtil.onRequestPermissionsResult(requestCode, permissions, grantResults);
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
    }

    @Override
    public void onPermissionsDenied(int requestCode, String... permission) {

    }

    @Override
    public void onPermissionsGranted(int requestCode, String... permission) {

    }
}
