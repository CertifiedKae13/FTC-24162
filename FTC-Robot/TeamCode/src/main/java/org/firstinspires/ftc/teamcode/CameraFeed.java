package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.util.Log;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * Camera stream for the FTC Dashboard (its Camera view). If no camera is configured, plugged in, or openable, it
 * streams a BLACK frame instead, never an error, and everything else keeps running. Nothing here throws.
 */
public final class CameraFeed implements CameraStreamSource {
    private static final String TAG = "CameraFeed";
    /** Name of the webcam in the Robot Controller hardware configuration. */
    public static final String WEBCAM_NAME = "Webcam 1";
    private static final int WIDTH = 640, HEIGHT = 480;

    private static VisionPortal portal;
    private static boolean streaming;

    private final Bitmap black = Bitmap.createBitmap(WIDTH, HEIGHT, Bitmap.Config.ARGB_8888);
    private CameraFeed() { black.eraseColor(Color.BLACK); }
 
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(black));
    }

    /** Start the Dashboard camera stream. Returns "camera" or "black screen" (for telemetry). Never throws. */
    public static String start(HardwareMap hardwareMap) {
        try {
            FtcDashboard dash = FtcDashboard.getInstance();
            if (dash == null) return "black screen";
            WebcamName cam = hardwareMap.tryGet(WebcamName.class, WEBCAM_NAME);
            if (cam != null && cam.isAttached()) {
                try {
                    portal = new VisionPortal.Builder().setCamera(cam)
                            .setCameraResolution(new Size(WIDTH, HEIGHT)).build();
                    dash.startCameraStream(portal, AutoConfig.CAMERA_FPS);
                    streaming = true;
                    return "camera";
                } catch (Throwable t) {          // camera present but would not open: fall through to black
                    Log.w(TAG, "camera failed, showing black screen: " + t);
                    closePortal();
                }
            }
            dash.startCameraStream(new CameraFeed(), AutoConfig.CAMERA_FPS);
            streaming = true;
            return "black screen";
        } catch (Throwable t) {
            Log.w(TAG, "camera stream unavailable: " + t);
            return "black screen";
        }
    }

    public static void stop() {
        try {
            if (streaming) {
                FtcDashboard dash = FtcDashboard.getInstance();
                if (dash != null) dash.stopCameraStream();
            }
        } catch (Throwable t) {
            Log.w(TAG, "stopCameraStream: " + t);
        }
        streaming = false;
        closePortal();
    }

    private static void closePortal() {
        try { if (portal != null) portal.close(); } catch (Throwable ignored) { }
        portal = null;
    }
}
