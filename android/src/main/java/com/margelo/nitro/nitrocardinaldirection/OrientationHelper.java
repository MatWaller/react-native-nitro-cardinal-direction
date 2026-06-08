package com.margelo.nitro.nitrocardinaldirection;

import android.content.Context;
import android.view.WindowManager;
import android.view.Display;
import android.os.Build;
import android.view.Surface;

public class OrientationHelper {
    private static Context appContext;

    public static void initialize(Context context) {
        appContext = context.getApplicationContext();
    }

    @SuppressWarnings("deprecation")
    public static int getDisplayRotation() {
        if (appContext == null) return Surface.ROTATION_0;

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            Display display = appContext.getDisplay();
            return display != null ? display.getRotation() : Surface.ROTATION_0;
        }

        WindowManager wm = (WindowManager) appContext.getSystemService(Context.WINDOW_SERVICE);
        if (wm == null) return Surface.ROTATION_0;
        Display display = wm.getDefaultDisplay();
        if (display == null) return Surface.ROTATION_0;
        return display.getRotation();
    }
}
