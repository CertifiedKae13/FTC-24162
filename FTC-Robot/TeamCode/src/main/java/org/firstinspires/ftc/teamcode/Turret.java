package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Turret rotation driven by Limelight tracking + PID, with manual overrides. */
public class Turret {
    private static final double KP = 0.033;
    private static final double KI = 0.002;
    private static final double KD = 0.005;
    private static final double I_CLAMP = 0.30;

    private static final double DEADBAND_DEG = 3.5;
    private static final double MAX_TRACK_SPEED = 1.0;
    private static final double MANUAL_SPD = 1.0;
    private static final double FEED_FWD_GAIN = 0.6;
    private static final long DETECT_TIMEOUT = 250;

    private final CRServo turretServo;
    private final Limelight3A limelight;
    private final ElapsedTime detectTimer = new ElapsedTime();

    private TurretMode turretMode = TurretMode.IDLE;
    private boolean autoTrack = true;
    private double pidIntegral = 0;
    private double pidPrevErr = 0;
    private double lastTx = 0;

    public Turret(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(CRServo.class, "rotationServo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }

    public void update(Gamepad curr, Gamepad prev, double dt) {
        if (GamepadUtil.rising(curr.back, prev.back)) autoTrack = !autoTrack;

        boolean tagSeen = readLimelight();

        double manual = 0;
        if (curr.right_trigger > 0.05) manual = curr.right_trigger * MANUAL_SPD;
        else if (curr.left_trigger > 0.05) manual = -curr.left_trigger * MANUAL_SPD;

        boolean wantsManual = Math.abs(manual) > 0.01;
        double ff = -curr.right_stick_x * FEED_FWD_GAIN;
        double cmd;

        if (!autoTrack) {
            cmd = wantsManual ? manual : 0;
            turretMode = TurretMode.MANUAL;
            resetPID();
        } else if (tagSeen) {
            double pid = computePID(lastTx, dt);
            cmd = pid + ff;
            turretMode = Math.abs(lastTx) <= DEADBAND_DEG ? TurretMode.LOCKED : TurretMode.TRACKING;
        } else if (wantsManual) {
            cmd = manual + ff;
            turretMode = TurretMode.MANUAL;
            resetPID();
        } else {
            cmd = ff;
            turretMode = Math.abs(ff) > 0.01 ? TurretMode.HOLDING : TurretMode.IDLE;
            resetPID();
        }

        turretServo.setPower(Range.clip(cmd, -1.0, 1.0));
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("── 操你妈 ──");
        telemetry.addData(" Mode", turretMode);
        telemetry.addData(" Auto", autoTrack ? "ON" : "off");
        telemetry.addData(" Tx", "%.2f°", lastTx);
    }

    private boolean readLimelight() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getStaleness() < DETECT_TIMEOUT) {
            double tx = result.getTx();
            if (tx != 0.0 || result.getTy() != 0.0) {
                lastTx = tx;
                detectTimer.reset();
            }
        }
        boolean recent = detectTimer.milliseconds() < DETECT_TIMEOUT;
        if (!recent) resetPID();
        return recent;
    }

    private double computePID(double error, double dt) {
        if (Math.abs(error) <= DEADBAND_DEG) {
            pidIntegral = 0;
            pidPrevErr = error;
            return 0;
        }
        pidIntegral = Range.clip(pidIntegral + error * dt, -I_CLAMP, I_CLAMP);
        double deriv = dt > 0 ? (error - pidPrevErr) / dt : 0;
        pidPrevErr = error;
        return Range.clip(
                KP * error + KI * pidIntegral + KD * deriv,
                -MAX_TRACK_SPEED, MAX_TRACK_SPEED);
    }

    private void resetPID() {
        pidIntegral = 0;
        pidPrevErr = 0;
    }
}
