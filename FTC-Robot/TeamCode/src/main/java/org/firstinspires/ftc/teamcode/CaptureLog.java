package org.firstinspires.ftc.teamcode;

import android.util.Log;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Records a run (pose, target, goal, battery) to a CSV on the hub so it can be replayed on the Dashboard with the
 * "Replay Last Auto" OpMode, or pulled with: adb pull /sdcard/FIRST/capture . Never throws: if the file cannot be
 * written it just turns itself off.
 */
public final class CaptureLog {
    private static final String TAG = "CaptureLog";
    public static final String HEADER = "t_ms,est_x,est_y,est_heading,target_x,target_y,goal_x,goal_y,battery_v";

    private FileWriter out;
    private long startMs;
    private long lastMs = -1000;

    public static File folder() { return new File(AppUtil.FIRST_FOLDER, "capture"); }

    /** Open a new capture file, or return an inert log if capturing is off or the file cannot be created. */
    public static CaptureLog start() {
        CaptureLog log = new CaptureLog();
        if (!AutoConfig.CAPTURE_ENABLED) return log;
        try {
            File dir = folder();
            //noinspection ResultOfMethodCallIgnored
            dir.mkdirs();
            File f = new File(dir, "auto-" + new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date()) + ".csv");
            log.out = new FileWriter(f);
            log.out.write(HEADER + "\n");
            log.startMs = System.currentTimeMillis();
        } catch (IOException | RuntimeException e) {
            Log.w(TAG, "capture off: " + e);
            log.out = null;
        }
        return log;
    }

    /** Append one row (throttled to AutoConfig.CAPTURE_HZ). */
    public void record(double ex, double ey, double eh, double tx, double ty, double gx, double gy, double batteryV) {
        if (out == null) return;
        long now = System.currentTimeMillis() - startMs;
        if (now - lastMs < 1000.0 / Math.max(1.0, AutoConfig.CAPTURE_HZ)) return;
        lastMs = now;
        try {
            out.write(String.format(Locale.US, "%d,%.3f,%.3f,%.2f,%.3f,%.3f,%.3f,%.3f,%.2f\n",
                    now, ex, ey, eh, tx, ty, gx, gy, batteryV));
        } catch (IOException | RuntimeException e) {
            Log.w(TAG, "capture stopped: " + e);
            close();
        }
    }

    public void close() {
        try { if (out != null) out.close(); } catch (IOException ignored) { }
        out = null;
    }
}
