package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Turns a ShotSolver solution into hardware commands: flywheel "launchMotor" (velocity control) and hood
 * "launchServo" (ASSUMED to set the launch angle; see ShotConfig.HOOD_*). Also tracks the hive tilt between tag
 * sightings. Never feeds a ball: firing stays with the driver / caller once atSpeed() is true.
 */
public class Shooter {
    private final DcMotorEx flywheel;
    private final Servo hood;
    private double targetRpm = 0;

    // Hive tilt memory (estimate 1): last tag tilt and when it was seen
    private double lastTiltDeg = Double.NaN, lastTiltTime = -1;

    public Shooter(HardwareMap hw) {
        flywheel = hw.tryGet(DcMotorEx.class, "launchMotor");
        hood = hw.tryGet(Servo.class, "launchServo");
        if (flywheel != null) {
            flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public boolean hasHardware() { return flywheel != null && hood != null; }

    /** Tilt of OUR hive: this frame's tag tilt if the tags are on our hive, else held, else NaN (assumed). */
    public Object[] tilt(LimelightVision.Reading r, boolean red, double now) {
        if (r != null && r.valid && r.hasHiveTilt && r.hiveIsRed == red && !Double.isNaN(r.hiveTiltDeg)) {
            lastTiltDeg = r.hiveTiltDeg;
            lastTiltTime = now;
            return new Object[]{lastTiltDeg, true, "tags"};
        }
        if (lastTiltTime >= 0 && now - lastTiltTime <= ShotConfig.TILT_HOLD_S) {
            return new Object[]{lastTiltDeg, true, String.format("held %.1fs", now - lastTiltTime)};
        }
        return new Object[]{Double.NaN, false, "assumed"};
    }

    /**
     * One call per loop: all three estimates -> solution (does not move anything). `r` must come from
     * LimelightVision.update(heading) so it carries the tag pose.
     */
    public ShotSolver.Solution solve(boolean red, Odometry2 odo, LimelightVision.Reading r, double now) {
        Object[] t = tilt(r, red, now);
        boolean ready = odo != null && odo.isReady();
        boolean tagPose = r != null && r.valid && r.hasPose;   // from the tags (or a Limelight field map)
        double range = r != null && r.tagCluster >= 0
                ? Math.sqrt(r.tagFwdIn * r.tagFwdIn + r.tagRightIn * r.tagRightIn + r.tagUpIn * r.tagUpIn) : Double.NaN;
        return ShotSolver.solve(red,
                ready ? odo.getX() : Double.NaN, ready ? odo.getY() : Double.NaN,
                odo != null ? odo.getHeadingDeg() : Double.NaN,
                (Double) t[0], (Boolean) t[1], (String) t[2],
                tagPose ? r.x : Double.NaN, tagPose ? r.y : Double.NaN, range,
                r != null && r.hasHiveTilt);
    }

    /** Set hood + flywheel for a solution. Returns false (and spins down) if there is no valid solution. */
    public boolean apply(ShotSolver.Solution s) {
        if (!hasHardware() || s == null || !s.ok) { stop(); return false; }
        hood.setPosition(ShotSolver.hoodPosition(s.angleDeg));
        targetRpm = s.rpm;
        flywheel.setVelocity(targetRpm / 60.0 * flywheel.getMotorType().getTicksPerRev());
        return true;
    }

    public double rpm() {
        return flywheel == null ? Double.NaN : flywheel.getVelocity() * 60.0 / flywheel.getMotorType().getTicksPerRev();
    }

    /** Within 3% of the commanded RPM (same tolerance as LaunchTester). */
    public boolean atSpeed() {
        return targetRpm > 0 && Math.abs(rpm() - targetRpm) <= 0.03 * targetRpm;
    }

    public void stop() {
        targetRpm = 0;
        if (flywheel != null) flywheel.setVelocity(0);
    }
}
