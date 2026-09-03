package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Odometry-based autonomous helpers built on the goBILDA Pinpoint
 * (odometry pods + built-in IMU), wired to I2C bus 0.
 *
 * Coordinate / direction conventions (match the teleop sticks):
 *   fwd    = +1  →  forward   (left stick up)
 *   strafe = +1  →  left      (left stick left)
 *   turn   = +1  →  rotate left (right stick left)
 *
 * Pinpoint field frame (anchored at reset): +X = forward, +Y = left,
 * relative to the robot's heading at the moment resetPosAndIMU() ran.
 */
public class Odometry {

    // ─────────────────────────────────────────────────────────
    // TUNING CONSTANTS  (measure / tune these)
    // ─────────────────────────────────────────────────────────

    // Odometry pod offsets relative to the tracking point, in MM.
    // TODO: measure your own robot. Left/forward of the tracking point are positive.
    private static final double POD_X_OFFSET_MM  = -84.0;  // X (forward) pod sideways offset
    private static final double POD_Y_OFFSET_MM  = -168.0; // Y (strafe) pod forward offset

    // Turn PID (heading loop). Feedback in degrees, output in motor power.
    private static final double TURN_kP = 0.02;
    private static final double TURN_kI = 0.0;
    private static final double TURN_kD = 0.01;
    private static final double TURN_MAX_POWER     = 0.7;
    private static final double TURN_TOLERANCE_DEG = 2.0;

    // Drive PID (distance loop). Feedback in inches, output in motor power.
    private static final double DIST_kP = 0.02;
    private static final double DIST_kI = 0.0;
    private static final double DIST_kD = 0.0;
    private static final double DIST_MAX_SPEED    = 0.8;
    private static final double DIST_TOLERANCE_IN = 1.0;

    // Heading-hold PID (keeps heading constant while drifting).
    private static final double HOLD_kP = 0.03;
    private static final double HOLD_kI = 0.0;
    private static final double HOLD_kD = 0.0;
    private static final double HOLD_MAX_POWER = 0.5;

    /*
     * Turning sign. Teleop maps right-stick-left → turn = +1 (rotate left), and the
     * spec is "negative angle = left, positive angle = right".
     * If turnTo(-45) rotates RIGHT instead of left, flip this to -1.
     */
    private static final double TURN_DIRECTION = 1.0;

    // Safety timeouts (ms) so a stuck loop can never run forever.
    private static final double TURN_TIMEOUT_MS  = 3000;
    private static final double DRIVE_TIMEOUT_MS = 5000;

    // ─────────────────────────────────────────────────────────
    // STATE
    // ─────────────────────────────────────────────────────────

    private final GoBildaPinpointDriver pinpoint;
    private final Drivetrain drivetrain;

    private Pose2D pose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
    private double voltageScale = 1.0;

    public Odometry(HardwareMap hardwareMap, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    // ─────────────────────────────────────────────────────────
    // SETUP
    // ─────────────────────────────────────────────────────────

    /** Call once in init(), BEFORE waitForStart(). Robot must be stationary. */
    public void init() {
        pinpoint.setOffsets(POD_X_OFFSET_MM, POD_Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();   // zero position + recalibrate IMU (must be still!)
    }

    /** Optional: scale motor power by battery voltage (like teleop). */
    public void setVoltageScale(double scale) {
        voltageScale = scale;
    }

    /** Re-zero position + heading. Call while stationary (e.g. start of auton). */
    public void reset() {
        pinpoint.resetPosAndIMU();
        update();
    }

    // ─────────────────────────────────────────────────────────
    // READINGS
    // ─────────────────────────────────────────────────────────

    /** Must be called once per loop iteration before reading. */
    public void update() {
        pinpoint.update();
        pose = pinpoint.getPosition();
    }

    public double getX() {
        return pose.getX(DistanceUnit.INCH);
    }

    public double getY() {
        return pose.getY(DistanceUnit.INCH);
    }

    public double getHeadingDeg() {
        return pose.getHeading(AngleUnit.DEGREES);
    }

    // ─────────────────────────────────────────────────────────
    // FUNCTION 1 — turnTo
    // ─────────────────────────────────────────────────────────

    /**
     * Turn by a relative angle. Negative = left, positive = right.
     * Blocks until the turn is complete (or times out).
     */
    public void turnTo(LinearOpMode opMode, double angleDeg) {
        update();
        double target = wrapDeg(getHeadingDeg() + angleDeg);

        PID pid = new PID(TURN_kP, TURN_kI, TURN_kD);
        ElapsedTime timer = new ElapsedTime();
        int settled = 0;

        while (opMode.opModeIsActive() && timer.milliseconds() < TURN_TIMEOUT_MS) {
            update();
            double error = wrapDeg(target - getHeadingDeg());
            double turn = TURN_DIRECTION
                    * Range.clip(pid.calculate(error), -TURN_MAX_POWER, TURN_MAX_POWER);

            drivetrain.drive(0, 0, turn, voltageScale);

            if (Math.abs(error) < TURN_TOLERANCE_DEG) {
                settled++;
                if (settled >= 5) break;   // stable for 5 consecutive frames
            } else {
                settled = 0;
            }
        }
        drivetrain.drive(0, 0, 0, voltageScale);
    }

    // ─────────────────────────────────────────────────────────
    // FUNCTION 2 — driveTo
    // ─────────────────────────────────────────────────────────

    /**
     * Field-centric drift to (x, y), keeping the current heading constant.
     * Distance PID gives "accelerate when far, decelerate when close";
     * heading-hold PID keeps the robot from rotating while strafing.
     */
    public void driveTo(LinearOpMode opMode, double targetX, double targetY) {
        update();
        double holdHeading = getHeadingDeg();

        PID distPid = new PID(DIST_kP, DIST_kI, DIST_kD);
        PID holdPid = new PID(HOLD_kP, HOLD_kI, HOLD_kD);
        ElapsedTime timer = new ElapsedTime();

        while (opMode.opModeIsActive() && timer.milliseconds() < DRIVE_TIMEOUT_MS) {
            update();

            double errX = targetX - getX();
            double errY = targetY - getY();
            double dist = Math.hypot(errX, errY);

            if (dist < DIST_TOLERANCE_IN) break;

            // Position loop → speed (P term: fast when far, slow when near).
            double speed = Range.clip(distPid.calculate(dist), 0, DIST_MAX_SPEED);

            // Field-frame direction toward the target.
            double dirX = errX / dist;
            double dirY = errY / dist;

            // Rotate field-frame velocity into the robot frame using current heading.
            double theta = Math.toRadians(getHeadingDeg());
            double cos = Math.cos(theta);
            double sin = Math.sin(theta);
            double fwd    = speed * (dirX * cos + dirY * sin);
            double strafe = speed * (-dirX * sin + dirY * cos);

            // Heading hold → turn correction (keeps robot facing holdHeading).
            double headingErr = wrapDeg(holdHeading - getHeadingDeg());
            double turn = TURN_DIRECTION
                    * Range.clip(holdPid.calculate(headingErr), -HOLD_MAX_POWER, HOLD_MAX_POWER);

            drivetrain.drive(fwd, strafe, turn, voltageScale);
        }
        drivetrain.drive(0, 0, 0, voltageScale);
    }

    // ─────────────────────────────────────────────────────────
    // HELPERS
    // ─────────────────────────────────────────────────────────

    /** Normalize an angle to [-180, 180] so turns always take the short way. */
    private static double wrapDeg(double deg) {
        while (deg > 180)  deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    /** Minimal PID controller with basic integral anti-windup. */
    private static class PID {
        private final double kP, kI, kD;
        private double integral = 0;
        private double prevError = 0;
        private double prevTime = -1;

        PID(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        double calculate(double error) {
            double now = System.nanoTime() / 1e9;
            double dt = prevTime < 0 ? 0 : now - prevTime;
            prevTime = now;

            integral += error * dt;
            integral = Range.clip(integral, -1.0, 1.0);   // anti-windup clamp

            double derivative = dt > 1e-6 ? (error - prevError) / dt : 0;
            prevError = error;

            return kP * error + kI * integral + kD * derivative;
        }
    }
}
