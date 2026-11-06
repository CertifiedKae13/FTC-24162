package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Pineapple2 diagnostics.
 *
 * Writes cached controller state to a CSV file in the FTC Robot Controller data
 * directory. It deliberately does not read hardware itself, so enabling logging
 * cannot add extra Pinpoint/I2C reads or change control-loop sensor ordering.
 */
public final class TelemetryCapture2 {
    private static final double SAMPLE_INTERVAL_S = 0.05; // 20 Hz
    private static final int FLUSH_EVERY_SAMPLES = 20;    // ~1 second

    private BufferedWriter writer;
    private File file;
    private boolean healthy;
    private String status = "NOT STARTED";
    private double lastSampleTime = -1.0;
    private int samplesSinceFlush;

    public TelemetryCapture2() {
        open();
    }

    private void open() {
        try {
            AppUtil.getInstance().ensureDirectoryExists(AppUtil.ROBOT_DATA_DIR);
            String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US)
                    .format(new Date());
            file = new File(AppUtil.ROBOT_DATA_DIR,
                    "Pineapple2Telemetry-" + stamp + ".csv");
            writer = new BufferedWriter(new FileWriter(file, false), 16384);
            writer.write("time_s,mode,pinpoint_status,ready,motion_usable,nav_state,reset_state,last_result,"
                    + "pose_x_in,pose_y_in,heading_deg,target_x_in,target_y_in,target_heading_deg,"
                    + "error_x_in,error_y_in,distance_error_in,heading_error_deg,"
                    + "velocity_x_ips,velocity_y_ips,angular_velocity_dps,"
                    + "field_command_x,field_command_y,"
                    + "robot_forward_cmd,robot_right_cmd,robot_cw_cmd,"
                    + "front_left_power,front_right_power,back_left_power,back_right_power,"
                    + "turn_p,turn_d,turn_ff,turn_raw,turn_ccw_out,turn_clipped,control_dt_s,"
                    + "manual_assist,stick_left_y,stick_left_x,stick_right_x,"
                    + "dpad_up,dpad_down,dpad_left,dpad_right,x,b,y,back,left_bumper,"
                    + "battery_v,voltage_scale\n");
            writer.flush();
            healthy = true;
            status = "LOGGING";
        } catch (IOException | RuntimeException ex) {
            healthy = false;
            writer = null;
            status = "OPEN FAILED: " + ex.getClass().getSimpleName();
        }
    }

    public void sample(double timeSeconds, String mode, Gamepad gamepad,
                       Drivetrain2 drivetrain, Odometry2 odometry,
                       double batteryVoltage, double voltageScale) {
        if (!healthy || writer == null || drivetrain == null || odometry == null
                || gamepad == null || !finite(timeSeconds)) return;
        if (lastSampleTime >= 0.0 && timeSeconds - lastSampleTime < SAMPLE_INTERVAL_S) return;
        lastSampleTime = timeSeconds;

        try {
            writer.write(String.format(Locale.US,
                    "%.4f,%s,%s,%s,%s,%s,%s,%s,"
                            + "%.6f,%.6f,%.3f,%.6f,%.6f,%.3f,"
                            + "%.6f,%.6f,%.6f,%.3f,"
                            + "%.6f,%.6f,%.3f,"
                            + "%.6f,%.6f,"
                            + "%.5f,%.5f,%.5f,"
                            + "%.5f,%.5f,%.5f,%.5f,"
                            + "%.5f,%.5f,%.5f,%.5f,%.5f,%s,%.5f,"
                            + "%s,%.4f,%.4f,%.4f,"
                            + "%s,%s,%s,%s,%s,%s,%s,%s,%s,"
                            + "%.3f,%.5f\n",
                    timeSeconds,
                    csv(mode),
                    csv(odometry.getStatus()),
                    odometry.isReady(),
                    odometry.isMotionEstimateUsable(),
                    csv(odometry.getNavigationState()),
                    csv(odometry.getResetState()),
                    csv(odometry.getLastResult()),
                    odometry.getX(),
                    odometry.getY(),
                    odometry.getHeadingDeg(),
                    odometry.getTargetX(),
                    odometry.getTargetY(),
                    odometry.getTargetHeading(),
                    odometry.getErrorX(),
                    odometry.getErrorY(),
                    odometry.getDistanceError(),
                    odometry.getHeadingErrorDeg(),
                    odometry.getVelocityX(),
                    odometry.getVelocityY(),
                    odometry.getAngularVelocityDegPerSec(),
                    odometry.getCommandX(),
                    odometry.getCommandY(),
                    drivetrain.getRequestedForward(),
                    drivetrain.getRequestedRight(),
                    drivetrain.getRequestedClockwise(),
                    drivetrain.getFrontLeftPower(),
                    drivetrain.getFrontRightPower(),
                    drivetrain.getBackLeftPower(),
                    drivetrain.getBackRightPower(),
                    odometry.getTurnP(),
                    odometry.getTurnD(),
                    odometry.getTurnFeedforward(),
                    odometry.getTurnRaw(),
                    odometry.getTurnCommand(),
                    odometry.isTurnClipped(),
                    odometry.getControlDt(),
                    csv(drivetrain.getManualAssistStatus()),
                    gamepad.left_stick_y,
                    gamepad.left_stick_x,
                    gamepad.right_stick_x,
                    gamepad.dpad_up,
                    gamepad.dpad_down,
                    gamepad.dpad_left,
                    gamepad.dpad_right,
                    gamepad.x,
                    gamepad.b,
                    gamepad.y,
                    gamepad.back,
                    gamepad.left_bumper,
                    batteryVoltage,
                    voltageScale));

            samplesSinceFlush++;
            if (samplesSinceFlush >= FLUSH_EVERY_SAMPLES) {
                writer.flush();
                samplesSinceFlush = 0;
            }
        } catch (IOException | RuntimeException ex) {
            healthy = false;
            status = "WRITE FAILED: " + ex.getClass().getSimpleName();
            closeWriterQuietly();
        }
    }

    /** Compact block intended to be readable in a Driver Station screen recording. */
    public void addDriverStationTelemetry(Telemetry telemetry, String mode, Gamepad gamepad,
                                          Drivetrain2 drivetrain, Odometry2 odometry) {
        if (telemetry == null || drivetrain == null || odometry == null || gamepad == null) return;
        telemetry.addData("LOG", "%s | %s", status, getFileName());
        telemetry.addData("NAV", "%s | %s", odometry.getNavigationState(), odometry.getLastResult());
        telemetry.addData("POSE", "X %.3f  Y %.3f  H %.1f",
                odometry.getX(), odometry.getY(), odometry.getHeadingDeg());
        telemetry.addData("TARGET", "X %.3f  Y %.3f  H %.1f",
                odometry.getTargetX(), odometry.getTargetY(), odometry.getTargetHeading());
        telemetry.addData("ERROR", "dX %.3f  dY %.3f  dist %.3f  dH %.1f",
                odometry.getErrorX(), odometry.getErrorY(),
                odometry.getDistanceError(), odometry.getHeadingErrorDeg());
        telemetry.addData("VELOCITY", "X %.3f  Y %.3f  H %.1f/s",
                odometry.getVelocityX(), odometry.getVelocityY(),
                odometry.getAngularVelocityDegPerSec());
        telemetry.addData("COMMAND", "field %.3f/%.3f | robot F %.3f R %.3f CW %.3f",
                odometry.getCommandX(), odometry.getCommandY(),
                drivetrain.getRequestedForward(), drivetrain.getRequestedRight(),
                drivetrain.getRequestedClockwise());
        telemetry.addData("WHEELS", "FL %.3f FR %.3f BL %.3f BR %.3f",
                drivetrain.getFrontLeftPower(), drivetrain.getFrontRightPower(),
                drivetrain.getBackLeftPower(), drivetrain.getBackRightPower());
        telemetry.addData("TURN", "P %.3f D %.3f FF %.3f OUTccw %.3f",
                odometry.getTurnP(), odometry.getTurnD(),
                odometry.getTurnFeedforward(), odometry.getTurnCommand());
        telemetry.addData("STICKS", "LY %.2f LX %.2f RX %.2f | %s",
                gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x,
                mode == null ? "" : mode);
    }

    public String getStatus() {
        return status;
    }

    public String getFileName() {
        return file == null ? "no file" : file.getName();
    }

    public String getFilePath() {
        return file == null ? "" : file.getAbsolutePath();
    }

    public void close() {
        if (writer == null) return;
        try {
            writer.flush();
        } catch (IOException ignored) {
            // Best effort during OpMode shutdown.
        }
        closeWriterQuietly();
        if (healthy) status = "CLOSED";
    }

    private void closeWriterQuietly() {
        if (writer == null) return;
        try {
            writer.close();
        } catch (IOException ignored) {
            // Best effort only.
        } finally {
            writer = null;
        }
    }

    private static String csv(String value) {
        if (value == null) return "";
        return value.replace(',', ';').replace('\n', ' ').replace('\r', ' ');
    }

    private static boolean finite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
