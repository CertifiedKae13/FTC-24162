package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * One-line blocking actions for autonomous and test OpModes, built on Odometry2's navigation.
 *
 * Units: inches and degrees. Field frame: origin at the field center (see FieldStart), heading 0 =
 * facing +X, CCW-positive. Every action returns true if it ARRIVED, false if it failed (timeout,
 * watchdog, hard cap, not ready) or the OpMode was stopped. A failed action stops the motors; the
 * caller should stop too ("if (!nav.drive(24)) return;").
 *
 * Usage in a LinearOpMode:
 *   NavActions nav = new NavActions(this);
 *   nav.init();            // calibrates Pinpoint; keep the robot still. Call before waitForStart().
 *   waitForStart();
 *   nav.drive(24); nav.turn(90); nav.strafe(-12); nav.driveTo(48, 0); nav.turnTo(180);
 */
public class NavActions {
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double MIN_VOLTAGE_SCALE = 0.80;
    private static final double MAX_VOLTAGE_SCALE = 1.15;
    private static final double LOOP_SLEEP_MS = 10;

    private final LinearOpMode opMode;
    private final Drivetrain2 drivetrain;
    private final Odometry2 odometry;
    private VoltageSensor voltageSensor;
    private String label = "";

    public NavActions(LinearOpMode opMode) {
        this.opMode = opMode;
        for (LynxModule hub : opMode.hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        if (opMode.hardwareMap.voltageSensor.iterator().hasNext()) {
            voltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
        }
        drivetrain = new Drivetrain2(opMode.hardwareMap);
        odometry = new Odometry2(opMode.hardwareMap, drivetrain);
    }

    /** Calibrate odometry (robot must be still) and wait until it is ready. Call before waitForStart(). */
    public void init() {
        opMode.telemetry.setMsTransmissionInterval(100);
        odometry.init();
        while (!opMode.isStarted() && !opMode.isStopRequested() && !odometry.isReady()) {
            odometry.update();
            odometry.updateReset();
            drivetrain.stop();
            opMode.telemetry.addLine("Calibrating Pinpoint: KEEP THE ROBOT STILL");
            odometry.addTelemetry(opMode.telemetry);
            opMode.telemetry.update();
            opMode.sleep((long) LOOP_SLEEP_MS);
        }
    }

    /** Cap all drive/turn power (0.1 to 1.0). Use a low value for first tests. */
    public void setSpeed(double scale) { odometry.setSpeedScale(scale); }

    /** Text shown in telemetry while an action runs (for example the current leg name). */
    public void setLabel(String text) { label = text; }

    // ---- Actions ----

    /** Drive forward (negative = backward) by inches along the current heading; heading is kept. */
    public boolean drive(double inches) {
        double h = Math.toRadians(odometry.getHeadingDeg());
        return go(odometry.getX() + inches * Math.cos(h), odometry.getY() + inches * Math.sin(h),
                odometry.getHeadingDeg());
    }

    /** Strafe RIGHT (negative = left) by inches relative to the robot; heading is kept. */
    public boolean strafe(double inches) {
        double h = Math.toRadians(odometry.getHeadingDeg());
        return go(odometry.getX() + inches * Math.sin(h), odometry.getY() - inches * Math.cos(h),
                odometry.getHeadingDeg());
    }

    /** Turn by degrees relative to the current heading. Positive = counter-clockwise. */
    public boolean turn(double degrees) {
        return go(odometry.getX(), odometry.getY(), odometry.getHeadingDeg() + degrees);
    }

    /** Go to field position (x, y) in inches, keeping the current heading. */
    public boolean driveTo(double x, double y) {
        return go(x, y, odometry.getHeadingDeg());
    }

    /** Go to field position (x, y) in inches and finish facing headingDeg (field, CCW-positive). */
    public boolean driveTo(double x, double y, double headingDeg) {
        return go(x, y, headingDeg);
    }

    /** Rotate in place to an absolute field heading in degrees (0 = +X, CCW-positive). */
    public boolean turnTo(double headingDeg) {
        return go(odometry.getX(), odometry.getY(), headingDeg);
    }

    /** Wait while keeping the robot stopped. Returns false if the OpMode stopped. */
    public boolean waitSeconds(double seconds) {
        long end = System.nanoTime() + (long) (seconds * 1e9);
        drivetrain.stop();
        while (opMode.opModeIsActive() && System.nanoTime() < end) {
            odometry.update();
            opMode.sleep((long) LOOP_SLEEP_MS);
        }
        return opMode.opModeIsActive();
    }

    public void stop() { odometry.cancelNavigation(); drivetrain.stop(); }

    public double getX() { return odometry.getX(); }
    public double getY() { return odometry.getY(); }
    public double getHeading() { return odometry.getHeadingDeg(); }
    public String getLastResult() { return odometry.getLastResult(); }

    // ---- Internals ----

    private boolean go(double x, double y, double headingDeg) {
        if (!opMode.opModeIsActive()) return false;
        odometry.update();
        if (!odometry.setTarget(x, y, headingDeg)) { drivetrain.stop(); return false; }
        odometry.startNavigation();
        while (opMode.opModeIsActive() && odometry.isNavigating()) {
            odometry.update();
            if (odometry.updateNavigation(voltageScale())) drivetrain.stop();
            showTelemetry();
            opMode.sleep((long) LOOP_SLEEP_MS);
        }
        if (odometry.isNavigating()) odometry.cancelNavigation(); // OpMode stopped mid-move
        drivetrain.stop();
        return odometry.hasArrived();
    }

    private double voltageScale() {
        if (voltageSensor == null) return 1.0;
        double v = voltageSensor.getVoltage();
        if (Double.isNaN(v) || Double.isInfinite(v) || v < 8.0 || v > 16.8) return 1.0;
        return Range.clip(NOMINAL_VOLTAGE / v, MIN_VOLTAGE_SCALE, MAX_VOLTAGE_SCALE);
    }

    private void showTelemetry() {
        if (!label.isEmpty()) opMode.telemetry.addData("Action", label);
        odometry.addTelemetry(opMode.telemetry);
        opMode.telemetry.update();
    }
}
