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
 * Odometry-based helpers built on the goBILDA Pinpoint.
 *
 * UNITS: METERS
 * Coordinates / direction conventions match the teleop sticks.
 */
public class Odometry {

    // ─────────────────────────────────────────────────────────
    // TUNING CONSTANTS (METRIC)
    // ─────────────────────────────────────────────────────────

    // Pod offsets in MM (Pinpoint handles mm conversion internally)
    private static final double POD_X_OFFSET_MM  = -84.0;
    private static final double POD_Y_OFFSET_MM  = -168.0;

    // Turn PID (Degrees)
    private static final double TURN_kP = 0.02;
    private static final double TURN_kI = 0.0;
    private static final double TURN_kD = 0.01;
    private static final double TURN_MAX_POWER     = 0.7;
    private static final double TURN_TOLERANCE_DEG = 2.0;

    // Drive PID (METERS)
    // kP needs to be higher because 1 meter error is much larger than 1 inch error
    private static final double DIST_kP = 0.8; 
    private static final double DIST_kI = 0.0;
    private static final double DIST_kD = 0.0;
    private static final double DIST_MAX_SPEED    = 0.6; // Reduced slightly for metric stability
    private static final double DIST_TOLERANCE_M  = 0.025; // ~1 inch in meters

    // Heading-hold PID
    private static final double HOLD_kP = 0.03;
    private static final double HOLD_kI = 0.0;
    private static final double HOLD_kD = 0.0;
    private static final double HOLD_MAX_POWER = 0.5;

    private static final double TURN_DIRECTION = 1.0;

    // Timeouts
    private static final double TURN_TIMEOUT_MS  = 3000;
    private static final double DRIVE_TIMEOUT_MS = 5000;

    // Puzzle Key Steps (METRIC)
    // 1 inch = 0.0254 meters
    private static final double PUZZLE_STEP_M = 0.0254; 
    private static final double PUZZLE_HEADING_STEP = 5.0;

    // ─────────────────────────────────────────────────────────
    // STATE
    // ─────────────────────────────────────────────────────────

    private final GoBildaPinpointDriver pinpoint;
    private final Drivetrain drivetrain;

    private Pose2D pose = new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0);
    private double voltageScale = 1.0;

    // Targets
    private double targetX = 0.0;
    private double targetY = 0.0;
    private double targetHeading = 0.0;
    
    // Navigation State
    private boolean isNavigating = false;
    private NavState navState = NavState.IDLE; // IDLE, TURNING, DRIVING
    
    // Button debouncing
    private boolean lastDPadUp = false;
    private boolean lastDPadDown = false;
    private boolean lastDPadLeft = false;
    private boolean lastDPadRight = false;
    private boolean lastYButton = false;
    private boolean lastXButton = false;
    private boolean lastBButton = false;

    // Internal timers for non-blocking loops
    private ElapsedTime navTimer = new ElapsedTime();
    private PID turnPid;
    private PID distPid;
    private PID holdPid;
    private double navHoldHeading = 0;

    private enum NavState { IDLE, TURNING, DRIVING }

    public Odometry(HardwareMap hardwareMap) {
        this.drivetrain = null; // Will be set via setter or constructor overload if needed
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        
        // Initialize PIDs
        turnPid = new PID(TURN_kP, TURN_kI, TURN_kD);
        distPid = new PID(DIST_kP, DIST_kI, DIST_kD);
        holdPid = new PID(HOLD_kP, HOLD_kI, HOLD_kD);
    }
    
    // Constructor if you need to pass drivetrain immediately
    public Odometry(HardwareMap hardwareMap, Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        turnPid = new PID(TURN_kP, TURN_kI, TURN_kD);
        distPid = new PID(DIST_kP, DIST_kI, DIST_kD);
        holdPid = new PID(HOLD_kP, HOLD_kI, HOLD_kD);
    }

    public void setDrivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    // ─────────────────────────────────────────────────────────
    // SETUP
    // ─────────────────────────────────────────────────────────

    public void init() {
        pinpoint.setOffsets(POD_X_OFFSET_MM, POD_Y_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
        
        update();
        targetX = getX();
        targetY = getY();
        targetHeading = getHeadingDeg();
    }

    public void setVoltageScale(double scale) {
        voltageScale = scale;
    }

    public void reset() {
        pinpoint.resetPosAndIMU();
        update();
        targetX = getX();
        targetY = getY();
        targetHeading = getHeadingDeg();
        isNavigating = false;
        navState = NavState.IDLE;
    }

    // ─────────────────────────────────────────────────────────
    // READINGS
    // ─────────────────────────────────────────────────────────

    public void update() {
        pinpoint.update();
        pose = pinpoint.getPosition();
    }

    public double getX() {
        return pose.getX(DistanceUnit.METER);
    }

    public double getY() {
        return pose.getY(DistanceUnit.METER);
    }

    public double getHeadingDeg() {
        return pose.getHeading(AngleUnit.DEGREES);
    }

    // ─────────────────────────────────────────────────────────
    // PUZZLE KEY CONTROLS
    // ─────────────────────────────────────────────────────────

    public void processInputs(boolean dpadUp, boolean dpadDown, boolean dpadLeft, boolean dpadRight,
                              boolean xBtn, boolean bBtn, boolean yBtn) {
        
        if (!isNavigating) {
            // Adjust Target Y
            if (dpadUp && !lastDPadUp) {
                targetY += PUZZLE_STEP_M;
            }
            if (dpadDown && !lastDPadDown) {
                targetY -= PUZZLE_STEP_M;
            }
            // Adjust Target X
            if (dpadRight && !lastDPadRight) {
                targetX += PUZZLE_STEP_M;
            }
            if (dpadLeft && !lastDPadLeft) {
                targetX -= PUZZLE_STEP_M;
            }
            
            // Adjust Target Heading
            if (xBtn && !lastXButton) {
                targetHeading -= PUZZLE_HEADING_STEP;
            }
            if (bBtn && !lastBButton) {
                targetHeading += PUZZLE_HEADING_STEP;
            }
        }

        // Start Navigation
        if (yBtn && !lastYButton && !isNavigating) {
            isNavigating = true;
            navState = NavState.TURNING; // Always turn first
            navTimer.reset();
            turnPid.reset();
            distPid.reset();
            holdPid.reset();
        }

        // Update Debounce States
        lastDPadUp = dpadUp;
        lastDPadDown = dpadDown;
        lastDPadLeft = dpadLeft;
        lastDPadRight = dpadRight;
        lastYButton = yBtn;
        lastXButton = xBtn;
        lastBButton = bBtn;
    }

    // ─────────────────────────────────────────────────────────
    // NON-BLOCKING NAVIGATION LOOP
    // Call this every cycle in TeleOp loop
    // ─────────────────────────────────────────────────────────

    /**
     * Executes one step of the navigation sequence.
     * @return true if navigation is complete or idle, false if still moving.
     */
    public boolean updateNavigation() {
        if (!isNavigating || drivetrain == null) {
            if(isNavigating && drivetrain == null) {
                // Safety break if drivetrain missing
                isNavigating = false; 
                drivetrain.drive(0,0,0, voltageScale);
            }
            return true;
        }

        update(); // Refresh odometry

        switch (navState) {
            case TURNING:
                handleTurningPhase();
                break;
            case DRIVING:
                handleDrivingPhase();
                break;
            case IDLE:
                return true;
        }
        return false;
    }

    private void handleTurningPhase() {
        double currentHeading = getHeadingDeg();
        double error = wrapDeg(targetHeading - currentHeading);

        if (Math.abs(error) < TURN_TOLERANCE_DEG) {
            // Turn complete, transition to driving
            navState = NavState.DRIVING;
            navHoldHeading = targetHeading; // Lock in the heading we just turned to
            navTimer.reset();
            // Reset PIDs for next phase
            distPid.reset();
            holdPid.reset();
        } else {
            // Execute Turn
            if (navTimer.milliseconds() > TURN_TIMEOUT_MS) {
                navState = NavState.DRIVING; // Timeout, move on
                return;
            }
            
            double turnPower = TURN_DIRECTION * Range.clip(turnPid.calculate(error), -TURN_MAX_POWER, TURN_MAX_POWER);
            drivetrain.drive(0, 0, turnPower, voltageScale);
        }
    }

    private void handleDrivingPhase() {
        double errX = targetX - getX();
        double errY = targetY - getY();
        double dist = Math.hypot(errX, errY);

        if (dist < DIST_TOLERANCE_M) {
            // Arrived
            isNavigating = false;
            navState = NavState.IDLE;
            drivetrain.drive(0, 0, 0, voltageScale);
            return;
        }

        if (navTimer.milliseconds() > DRIVE_TIMEOUT_MS) {
            isNavigating = false;
            navState = NavState.IDLE;
            drivetrain.drive(0, 0, 0, voltageScale);
            return;
        }

        // Calculate Drive Vector
        double speed = Range.clip(distPid.calculate(dist), 0, DIST_MAX_SPEED);
        double dirX = errX / dist;
        double dirY = errY / dist;

        double theta = Math.toRadians(getHeadingDeg());
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        
        // Field Centric to Robot Centric
        double fwd    = speed * (dirX * cos + dirY * sin);
        double strafe = speed * (-dirX * sin + dirY * cos);

        // Heading Hold
        double headingErr = wrapDeg(navHoldHeading - getHeadingDeg());
        double turn = TURN_DIRECTION * Range.clip(holdPid.calculate(headingErr), -HOLD_MAX_POWER, HOLD_MAX_POWER);

        drivetrain.drive(fwd, strafe, turn, voltageScale);
    }

    // ─────────────────────────────────────────────────────────
    // HELPERS & GETTERS
    // ─────────────────────────────────────────────────────────

    public void adjustTargetX(double delta) { targetX += delta; }
    public void adjustTargetY(double delta) { targetY += delta; }
    public void adjustTargetHeading(double delta) { targetHeading += delta; }

    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }
    public double getTargetHeading() { return targetHeading; }
    public boolean isNavigating() { return isNavigating; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- ODOMETRY (METERS) ---");
        telemetry.addData("Pos X", "%.3f m", getX());
        telemetry.addData("Pos Y", "%.3f m", getY());
        telemetry.addData("Heading", "%.1f°", getHeadingDeg());
        telemetry.addLine("--- TARGETS ---");
        telemetry.addData("Target X", "%.3f m", targetX);
        telemetry.addData("Target Y", "%.3f m", targetY);
        telemetry.addData("Target Hd", "%.1f°", targetHeading);
        telemetry.addData("Status", isNavigating ? "NAVIGATING" : "IDLE");
    }

    private static double wrapDeg(double deg) {
        while (deg > 180)  deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    private static class PID {
        private final double kP, kI, kD;
        private double integral = 0;
        private double prevError = 0;
        private double prevTime = -1;

        PID(double kP, double kI, double kD) {
            this.kP = kP; this.kI = kI; this.kD = kD;
        }

        void reset() {
            integral = 0;
            prevError = 0;
            prevTime = -1;
        }

        double calculate(double error) {
            double now = System.nanoTime() / 1e9;
            double dt = prevTime < 0 ? 0 : now - prevTime;
            prevTime = now;

            integral += error * dt;
            integral = Range.clip(integral, -1.0, 1.0);

            double derivative = dt > 1e-6 ? (error - prevError) / dt : 0;
            prevError = error;

            return kP * error + kI * integral + kD * derivative;
        }
    }
}