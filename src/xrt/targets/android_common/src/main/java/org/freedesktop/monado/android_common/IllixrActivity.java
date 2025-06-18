package org.freedesktop.monado.android_common;

import android.app.Activity;
import android.os.Bundle;
import android.util.Log;
import android.view.Surface;

import androidx.annotation.NonNull;

public class IllixrActivity extends Activity {

//    static {
//        // Load the shared library with the native parts of this class
//        // This is the service-lib target.
//        System.loadLibrary("monado-service");
//    }


    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        Log.i("illixr activity", "on create activity");
    }

   // private native void nativeAppSurface_Illixr(@NonNull Surface surface);
}
