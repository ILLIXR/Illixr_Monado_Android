package org.freedesktop.monado.ipc;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.Window;

import androidx.annotation.NonNull;

public class IllixrImpl extends Activity {
    private SurfaceManager surfaceManager;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        //setContentView(R.layout.activity_main);
        Log.d("on create", "illixr .. start");
//        this.surfaceManager = new SurfaceManager(this);
//        this.surfaceManager.setCallback(new SurfaceHolder.Callback() {
//            @Override
//            public void surfaceCreated(@NonNull SurfaceHolder holder) {
//                Log.i("illixrimpl", "surfaceCreated illixr impl");
//                //Intent myIntent = new Intent(this, IllixrImpl.class);
//                //passAppSurface(holder.getSurface());
//            }
//
//            @Override
//            public void surfaceChanged(@NonNull SurfaceHolder holder, int format, int width, int height) {
//            }
//
//            @Override
//            public void surfaceDestroyed(@NonNull SurfaceHolder holder) {
//            }
//        });
        Log.d("on create", "The onCreate() event");

    }
}
