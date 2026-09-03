package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Odometry-based navigation helpers built on the goBILDA Pinpoint.
 *
 * UNITS: metres, degrees.
 *
 * PINPOINT FRAME (fixed, cannot be changed):
 *   +X = robot forward, +Y = robot LEFT, heading CCW-positive (0° = start heading).
 *
 * The drivetrain's sign conventions are almost certainly different
 * (gamepad style: +strafe = right, +turn = clockwise), so they are mapped
 * with STRAFE_DIRECTION / TURN_DIRECTION below.  Verify these on the robot first.
 */
public class Odometry {

    // ─────────────────────────────────────────────────────────
    // TUNING CONSTANTS
    // ─────────────────────────────────────────────────────────

    /** Pod offsets from the robot's tracking point (mm).
     *  X pod (forward encoder):  distance LEFT (+) / right (−) of centre.
     *  Y pod (strafe encoder):   distance FORWARD (+) / back (−) of centre. */
    private static final double POD_X_OFFSET_MM = -84.0;
    private static final double POD_Y_OFFSET_MM = -168.0;

    /** Direction mapping between Pinpoint frame and Drivetrain.drive().
     *  +1: drive()'s positive strafe = LEFT   (matches Pinpoint)   | −1: positive strafe = RIGHT (gamepad style)
     *  +1: drive()'s positive turn   = CCW    (matches Pinpoint)   | −1: positive turn   = CW    (gamepad style)
     *  If the robot drives AWAY from the target, flip the relevant sign. */
    private static final double STRAFE_DIRECTION = -1.0;
    private static final double TURN_DIRECTION   = -1.0;

    // Turn PID (input: degrees)
    private static final double TURN_kP = 0.02;
    private static final double TURN_kI = 0.0;
    private static final double TURN_kD = 0.001;
    private static final double TURN_MAX_POWER     = 0.7;
    private static final double TURN_MIN_POWER     = 0.10;   // overcome static friction
    private static final double TURN_TOLERANCE_DEG = 2.0;

    // Drive PID (input: metres).
    // Numerically larger than an inch-based gain because 1 m of error is 39.37× bigger than 1 in.
    private static final double DIST_kP = 0.8;
    private static final double DIST_kI = 0.0;
    private static final double DIST_kD = 0.0;
    private static final double DIST_MAX_SPEED   = 0.6;
    private static final double DIST_MIN_SPEED   = 0.12;    // overcome static friction
    private static final double DIST_TOLERANCE_M = 0.015;   // 1.5 cm – must be < PUZZLE_STEP_M

    // Heading hold while driving
    private static final double HOLD_kP = 0.03;
    private static final double HOLD_kI = 0.0;
    private static final double HOLD_kD = 0.0;
    private static final double HOLD_MAX_POWER = 0.5;

    // Timeouts
    private static final double TURN_TIMEOUT_MS  = 3000;
    private static final double DRIVE_TIMEOUT_MS = 5000;

    // Puzzle-key steps
    private static final double PUZZLE_STEP_M        = 0.0254;  // 1 inch
    private static final double PUZZLE_HEADING_STEP  = 5.0;

    // ─────────────────────────────────────────────────────────
    // STATE
    // ─────────────────────────────────────────────────────────

    private enum NavState { IDLE, TURNING, DRIVING }

    private final GoBildaPinpointDriver pinpoint;
    private Drivetrain drivetrain;                       // not final: settable

    private Pose2D pose = new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0);

    private double targetX = 0.0;
    private double targetY = 0.0;
    private double targetHeading = 0.0;

    private NavState navState = NavState.IDLE;
    private String lastResult = "—";
    private double navHoldHeading = 0.0;

    private final ElapsedTime navTimer = new ElapsedTime();
    private final PID turnPid = new PID(TURN_kP, TURN_kI, TURN_kD);
    private final PID distPid = new PID(DIST_kP, DIST_kI, DIST_kD);
    private final PID holdPid = new PID(HOLD_kP, HOLD_kI, HOLD_kD);

    // ─────────────────────────────────────────────────────────
    // CONSTRUCTION / SETUP
    // ─────────────────────────────────────────────────────────

    public Odometry(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public Odometry(HardwareMap hardwareMap, Drivetrain drivetrain) {
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        this.drivetrain = drivetrain;
    }

    public void setDrivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    /** Call once in init(). Robot must be stationary (IMU calibrates). */
    public void init() {
        pinpoint.setOffsets(POD_X_OFFSET_MM, POD_Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // Verify: X pod must count UP when robot moves forward, Y pod UP when robot moves LEFT.
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        update();
        syncTargetsToPose();
    }

    /** Zero the pose and recalibrate the IMU. Robot must be stationary. */
    public void reset() {
        cancelNavigation();
        pinpoint.resetPosAndIMU();
        update();
        syncTargetsToPose();
    }

    // ─────────────────────────────────────────────────────────
    // READINGS  (call update() exactly once per loop)
    // ─────────────────────────────────────────────────────────

    public void update() {
        pinpoint.update();
        pose = pinpoint.getPosition();
    }

    public double getX()          { return pose.getX(DistanceUnit.METER); }
    public double getY()          { return pose.getY(DistanceUnit.METER); }
    public double getHeadingDeg() { return pose.getHeading(AngleUnit.DEGREES); }
    public String getStatus()     { return String.valueOf(pinpoint.getDeviceStatus()); }

    // ─────────────────────────────────────────────────────────
    // PUZZLE-KEY CONTROLS  (all arguments are rising-edge "pressed" events)
    // ─────────────────────────────────────────────────────────

    public void processInputs(boolean upPressed, boolean downPressed,
                              boolean leftPressed, boolean rightPressed,
                              boolean xPressed, boolean bPressed, boolean yPressed) {

        if (!isNavigating()) {
            if (upPressed)    targetY += PUZZLE_STEP_M;
            if (downPressed)  targetY -= PUZZLE_STEP_M;
            if (rightPressed) targetX += PUZZLE_STEP_M;
            if (leftPressed)  targetX -= PUZZLE_STEP_M;
            if (xPressed)     targetHeading = wrapDeg(targetHeading - PUZZLE_HEADING_STEP);
            if (bPressed)     targetHeading = wrapDeg(targetHeading + PUZZLE_HEADING_STEP);

            if (yPressed) startNavigation();
        }
    }

    public void startNavigation() {
        if (drivetrain == null) {
            lastResult = "NO DRIVETRAIN";
            return;
        }
        navState = NavState.TURNING;   // always turn first
        lastResult = "RUNNING";
        navTimer.reset();
        turnPid.reset();
        distPid.reset();
        holdPid.reset();
    }

    public void cancelNavigation() {
        if (isNavigating()) {
            lastResult = "CANCELLED";
            navState = NavState.IDLE;
        }
    }

    /** Make the current pose the target (so the next D-pad step is relative to here). */
    public void syncTargetsToPose() {
        targetX = getX();
        targetY = getY();
        targetHeading = wrapDeg(getHeadingDeg());
    }

    // ─────────────────────────────────────────────────────────
    // NON-BLOCKING NAVIGATION  (call every loop; update() must already have been called)
    // ─────────────────────────────────────────────────────────

    /**
     * Executes one navigation step.
     * @return true if idle/finished (caller should hold motors at zero), false if still moving.
     */
    public boolean updateNavigation(double voltageScale) {
        if (navState == NavState.IDLE) return true;

        if (drivetrain == null) {          // safety: cannot move without a drivetrain
            navState = NavState.IDLE;
            lastResult = "NO DRIVETRAIN";
            return true;
        }

        switch (navState) {
            case TURNING: handleTurningPhase(voltageScale); break;
            case DRIVING: handleDrivingPhase(voltageScale); break;
            default: break;
        }
        return navState == NavState.IDLE;
    }

    private void beginDrivingPhase(double holdHeading) {
        navState = NavState.DRIVING;
        navHoldHeading = holdHeading;
        navTimer.reset();
        distPid.reset();
        holdPid.reset();
    }

    private void finish(String result, double voltageScale) {
        navState = NavState.IDLE;
        lastResult = result;
        drivetrain.drive(0, 0, 0, voltageScale);
    }

    private void handleTurningPhase(double voltageScale) {
        double error = wrapDeg(targetHeading - getHeadingDeg());   // + = need CCW

        if (Math.abs(error) < TURN_TOLERANCE_DEG) {
            beginDrivingPhase(targetHeading);
            return;
        }
        if (navTimer.milliseconds() > TURN_TIMEOUT_MS) {
            lastResult = "TURN TIMEOUT";
            beginDrivingPhase(getHeadingDeg());   // hold whatever heading we reached
            return;
        }

        double raw = turnPid.calculate(error);
        double turn = Math.copySign(
                Range.clip(Math.abs(raw), TURN_MIN_POWER, TURN_MAX_POWER), error);

        drivetrain.drive(0, 0, TURN_DIRECTION * turn, voltageScale);
    }

    private void handleDrivingPhase(double voltageScale) {
        double errX = targetX - getX();
        double errY = targetY - getY();
        double dist = Math.hypot(errX, errY);

        if (dist < DIST_TOLERANCE_M) {
            finish("ARRIVED", voltageScale);
            return;
        }
        if (navTimer.milliseconds() > DRIVE_TIMEOUT_MS) {
            finish("DRIVE TIMEOUT", voltageScale);
            return;
        }

        // Speed magnitude (PID on distance, floored so we don't stall short of the target)
        double speed = Range.clip(distPid.calculate(dist), DIST_MIN_SPEED, DIST_MAX_SPEED);
        double dirX = errX / dist;
        double dirY = errY / dist;

        // Field frame → robot frame (rotate by −heading)
        double theta = Math.toRadians(getHeadingDeg());
        double cos = Math.cos(theta), sin = Math.sin(theta);
        double robotFwd  = speed * ( dirX * cos + dirY * sin);   // +forward
        double robotLeft = speed * (-dirX * sin + dirY * cos);   // +left (Pinpoint +Y)

        // Heading hold
        double headingErr = wrapDeg(navHoldHeading - getHeadingDeg());
        double turn = Range.clip(holdPid.calculate(headingErr), -HOLD_MAX_POWER, HOLD_MAX_POWER);

        drivetrain.drive(robotFwd,
                         STRAFE_DIRECTION * robotLeft,
                         TURN_DIRECTION * turn,
                         voltageScale);
    }

    // ─────────────────────────────────────────────────────────
    // GETTERS / TELEMETRY
    // ─────────────────────────────────────────────────────────

    public void adjustTargetX(double delta)       { targetX += delta; }
    public void adjustTargetY(double delta)       { targetY += delta; }
    public void adjustTargetHeading(double delta) { targetHeading = wrapDeg(targetHeading + delta); }

    public double  getTargetX()       { return targetX; }
    public double  getTargetY()       { return targetY; }
    public double  getTargetHeading() { return targetHeading; }
    public boolean isNavigating()     { return navState != NavState.IDLE; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- ODOMETRY (METRES) ---");
        telemetry.addData("Pinpoint", getStatus());
        telemetry.addData("Pos", "X %.3f  Y %.3f  H %.1f°", getX(), getY(), getHeadingDeg());
        telemetry.addLine("--- TARGET ---");
        telemetry.addData("Target", "X %.3f  Y %.3f  H %.1f°", targetX, targetY, targetHeading);
        telemetry.addData("Error", "%.3f m  %.1f°",
                Math.hypot(targetX - getX(), targetY - getY()),
                wrapDeg(targetHeading - getHeadingDeg()));
        telemetry.addData("Nav", "%s (%s)", navState, lastResult);
    }

    // ─────────────────────────────────────────────────────────
    // HELPERS
    // ─────────────────────────────────────────────────────────

    private static double wrapDeg(double deg) {
        return AngleUnit.normalizeDegrees(deg);   // → (−180, 180]
    }

    private static class PID {
        private final double kP, kI, kD;
        private double integral = 0;
        private double prevError = 0;
        private long prevTimeNs = -1;

        PID(double kP, double kI, double kD) { this.kP = kP; this.kI = kI; this.kD = kD; }

        void reset() { integral = 0; prevError = 0; prevTimeNs = -1; }

        double calculate(double error) {
            long now = System.nanoTime();
            double dt = prevTimeNs < 0 ? 0 : (now - prevTimeNs) / 1e9;
            prevTimeNs = now;

            integral = Range.clip(integral + error * dt, -1.0, 1.0);   // anti-windup
            double derivative = dt > 1e-6 ? (error - prevError) / dt : 0;
            prevError = error;

            return kP * error + kI * integral + kD * derivative;
        }
    }
}