package me.zhehua.gryostable;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.os.SystemClock;
import android.util.Log;

public class ThetaHelper implements SensorEventListener {

    public ThetaHelper() {
        n_init();
    }

    public static long timeConvert(long timestamp) {
        long nanoTime = System.nanoTime();
        long systemClock = SystemClock.elapsedRealtimeNanos();
        return timestamp + systemClock - nanoTime;
    }


    @Override
    public void onSensorChanged(SensorEvent event) {
        long timestamp = Camera2BasicFragment.isSensorUseRTCTime
                ? event.timestamp : timeConvert(event.timestamp);

        n_sensor_changed(timestamp, event.values[0], event.values[1], event.values[2]);
        Log.d("onSensorChanged", "onSensorChanged: x:"+event.values[0]+" y:"+ event.values[1] +
                " z:"+event.values[2]);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    private native void n_sensor_changed(long timestamp, float x, float y, float z);
    public native void n_getR(long timestamp, long matR, boolean isCrop);
    private native void n_init();
    public native void n_RsChangeVectorToMat(long rs_out_mat);
}
