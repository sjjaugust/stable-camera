package me.zhehua.gryostable;

import android.graphics.Matrix;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.util.Log;

import org.opencv.core.Mat;

import java.nio.Buffer;
import java.util.ArrayList;
import java.util.Queue;
import java.util.Stack;

public class JellyEffectRectify {
    private static final String TAG = "JellyEffectRectify";
    private long gyrotimestamp = 0;
    private static final float NS2S = 1.0f/1000000000.0f;
    public ArrayList<Float> curgyrox = new ArrayList<>();
    public ArrayList<Float> curgyroy = new ArrayList<>();
    public ArrayList<Float> curgyroz = new ArrayList<>();
    private long framecount = 0;
    private long lastframecount = 0;
    private float anglex = 0;
    private float angley = 0;
    private float anglez = 0;
    public long pictimestamp = 0;
    public ArrayList<Long> gyrots = new ArrayList<>();
    private long ts = 0;//扫过一帧所需时间
    private float fl = 0;//焦距
    public long timedelay = 0;


    SensorEventListener gyroscopeSensorListener = new SensorEventListener(){
        @Override
        public void onAccuracyChanged(Sensor sensor, int accurary){}

        @Override
        public void onSensorChanged(SensorEvent event){
            if(event.sensor.getType() == Sensor.TYPE_GYROSCOPE && framecount==lastframecount+1){
                long temp = event.timestamp;
                gyrots.add(temp);
                if(gyrotimestamp!=0.0f){
                    float dt = (temp-gyrotimestamp)*NS2S;
                    anglex += event.values[0]*dt;
                    angley += event.values[1]*dt;
                    anglez += event.values[2]*dt;
                    curgyrox.add(anglex);
                    curgyroy.add(angley);
                    curgyroz.add(anglez);
                    //Log.e(TAG, "onSensorChanged:"+curgyro.get(curgyro.size()-1)+' '+curgyro.get(curgyro.size()-2)+' '+curgyro.get(curgyro.size()-3));
                    //Log.e(TAG, "onSensorChanged: "+dt);
                    //Log.e(TAG, "onSensorChanged: "+curgyro.size());
                }
               gyrotimestamp = temp;
            }
            else {
//                Log.e(TAG, "onSensorChanged: "+curgyrox.size());
//                Log.e(TAG, "onSensorChanged:pictimestamp "+pictimestamp);
//                Log.e(TAG, "onSensorChanged:gyrotimestamp "+gyrots.size());
//                Log.e(TAG, "onSensorChanged: framecount"+framecount );
//                Log.e(TAG, "onSensorChanged: lastframecount"+lastframecount );
                curgyrox.clear();
                curgyroy.clear();
                curgyroz.clear();
                gyrots.clear();
                anglex = 0;
                angley = 0;
                anglez = 0;
                if(lastframecount == framecount-2){
                    lastframecount++;
                }

//                Log.e(TAG, "onSensorChanged: i am clear" );
            }
        }
    };

    public void ChangeRecordState(){
        framecount++;
    }

    public float interp1(long x0, ArrayList<Long> x, ArrayList<Float> y){
        long x1 = 0;
        long x2 = 0;
        float y1 = 0;
        float y2 = 0;
        float k = 0;
        float b = 0;
        for (int i = 1; i < x.size(); i++){
            if(x0<x.get(i)){
                x1 = x.get(i-1);
                x2 = x.get(i);
                y1 = y.get(i-1);
                y2 = y.get(i);
                k = (y2-y1)/(x2-x1);
                b = y1-k*x1;
                break;
            }
        }
        return (k*x0+b);
    }
    public void process(){
        ArrayList<Long> ft = new ArrayList<>();
        ft = linspace(pictimestamp-timedelay, pictimestamp+ts, 20);
       // Log.e(TAG, "processffff: "+ft );
        ArrayList<Float> curpicangle = new ArrayList<>();//当前帧全局曝光角度
        //Log.e(TAG, "processwtf: "+gyrots.size()+curgyrox.size());
        curpicangle.add(interp1(pictimestamp, gyrots, curgyrox));
        curpicangle.add(interp1(pictimestamp, gyrots, curgyroy));
        curpicangle.add(interp1(pictimestamp, gyrots, curgyroz));
        float[][] stripangle = new float[20][3];//每个条带的角度
        for(int i = 0; i < 20; i++){
            stripangle[i][0] = interp1(ft.get(i), gyrots, curgyrox);
            stripangle[i][1] = interp1(ft.get(i), gyrots, curgyroy);
            stripangle[i][2] = interp1(ft.get(i), gyrots, curgyroz);
        }
//        Log.e(TAG, "processsss: "+curgyrox);
    }
    public ArrayList<Long> linspace(long num1, long num2, int count){
        ArrayList<Long> re = new ArrayList<>();
        long gap = (num2-num1)/count;
        long num = num1;
        for(int i = 0; i < count; i++){
            re.add(num);
            num+=gap;
        }
        return re;
    }
}
