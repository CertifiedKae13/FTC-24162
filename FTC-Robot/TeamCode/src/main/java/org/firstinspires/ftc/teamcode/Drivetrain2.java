package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Robot frame: +forward, +strafe RIGHT, +turn CLOCKWISE. */
public class Drivetrain2 {
    private static final double STICK_DEADBAND = 0.10;
    private static final double STRAFE_SCALE = 1.1;
    private static final double MIN_VOLTAGE_SCALE = 0.80;
    private static final double MAX_VOLTAGE_SCALE = 1.15;

    // Manual heading assistance. Starting values: tune on the actual robot.
    private static final boolean HEADING_HOLD_ENABLED = true;
    private static final double MANUAL_HOLD_KP = 0.018; // power / degree
    private static final double MANUAL_HOLD_KD = 0.001; // power / (degree/s)
    private static final double MANUAL_HOLD_MAX = 0.18;
    private static final double TURN_RELEASE_DELAY_S = 0.15;
    private static final double RATE_FILTER_TIME_S = 0.08;
    private static final double MAX_MANUAL_GAP_S = 0.25;
    private static final double SETTLED_YAW_RATE_DPS = 15.0;   // capture hold heading only below this spin
    private static final double MAX_SETTLE_S = 0.5;

    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private double fwd, strafe, turn;
    private double frontLeftPower, frontRightPower, backLeftPower, backRightPower;
    private double appliedVoltageScale = 1.0;
    /** Largest |power| any wheel may get, after voltage compensation. 1.0 = no cap (autos); TeleOp lowers it. */
    private double maxWheelPower = 1.0;
    private boolean saturated;
    private String status = "STOPPED";
    private String manualAssistStatus = "OFF";
    private boolean manualTargetValid, headingSampleValid;
    private double manualTargetHeading, previousHeading, filteredYawRate;
    private double turnReleaseRemaining, manualHeadingError, manualCorrection, settleElapsed;

    public Drivetrain2(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        stop();
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /** Compatibility overload: ordinary driving without heading assistance. */
    public void update(Gamepad gamepad, double voltageScale) {
        update(gamepad, voltageScale, Double.NaN, 0.0);
    }

    /**
     * Manual driving only. headingDeg is the cached Pinpoint CCW-positive heading.
     * Pass NaN when odometry is not ready. dtSeconds is the actual loop interval.
     * Navigation must continue to call drive(), bypassing this manual controller.
     */
    public void update(Gamepad gamepad, double voltageScale,
                       double headingDeg, double dtSeconds) {
        if (gamepad == null || !finite(gamepad.left_stick_y)
                || !finite(gamepad.left_stick_x) || !finite(gamepad.right_stick_x)) {
            stop();
            status = "INVALID GAMEPAD";
            return;
        }
        double forward = shape(-gamepad.left_stick_y);
        double right = shape(gamepad.left_stick_x) * STRAFE_SCALE;
        double clockwise = shape(gamepad.right_stick_x);
        boolean driverTurning = Math.abs(gamepad.right_stick_x) > STICK_DEADBAND;
        double translation = Math.hypot(forward, right);

        if (!HEADING_HOLD_ENABLED || !finite(headingDeg) || !finite(dtSeconds)
                || dtSeconds <= 0.0 || dtSeconds > MAX_MANUAL_GAP_S) {
            resetManualHeadingHold();
            manualAssistStatus = !HEADING_HOLD_ENABLED ? "OFF"
                    : !finite(headingDeg) ? "NO HEADING" : "WAITING FOR LOOP";
        } else if (driverTurning || translation == 0.0) {
            // Use RAW turn input so even a small intentional turn releases hold.
            resetManualHeadingHold();
            turnReleaseRemaining = driverTurning ? TURN_RELEASE_DELAY_S : 0.0;
            manualAssistStatus = driverTurning ? "DRIVER TURN" : "OFF";
        } else if (turnReleaseRemaining > 0.0 || settlingSpin(headingDeg, dtSeconds)) {
            // Let a deliberate turn settle before capturing its final heading. Capturing while the robot
            // is still coasting locks a heading it then overshoots, and the hold snaps it back (drift).
            turnReleaseRemaining = Math.max(0.0, turnReleaseRemaining - dtSeconds);
            settleElapsed += dtSeconds;
            manualTargetValid = false;
            manualHeadingError = manualCorrection = 0.0;
            manualAssistStatus = "TURN SETTLING";
        } else {
            if (!manualTargetValid) {
                manualTargetHeading = normalize360(headingDeg);
                manualTargetValid = true;
            }
            if (headingSampleValid) {
                double rate = wrapSignedDeg(headingDeg - previousHeading) / dtSeconds;
                double alpha = dtSeconds / (RATE_FILTER_TIME_S + dtSeconds);
                filteredYawRate += alpha * (rate - filteredYawRate);
            } else {
                filteredYawRate = 0.0;
            }
            previousHeading = normalize360(headingDeg);
            headingSampleValid = true;
            manualHeadingError = wrapSignedDeg(manualTargetHeading - headingDeg);
            // Pinpoint is CCW-positive; drive() is CW-positive.
            // D acts on measured rotation, avoiding a target-change derivative kick.
            double correction = -MANUAL_HOLD_KP * manualHeadingError
                    + MANUAL_HOLD_KD * filteredYawRate;
            // Reduce assistance near the stick deadband to avoid strong turning
            // when the driver requests only a tiny amount of translation.
            double limit = MANUAL_HOLD_MAX * Math.min(1.0, translation / 0.25);
            manualCorrection = Range.clip(correction, -limit, limit);
            clockwise = manualCorrection;
            manualAssistStatus = "HEADING HOLD";
        }
        drive(forward, right, clockwise, voltageScale);
    }

    /** True while the robot is still rotating after a turn (bounded so hold always re-engages). */
    private boolean settlingSpin(double headingDeg, double dtSeconds) {
        if (manualTargetValid || settleElapsed >= MAX_SETTLE_S) return false;
        boolean firstSample = !headingSampleValid;
        double rate = 0.0;
        if (!firstSample) {
            rate = wrapSignedDeg(headingDeg - previousHeading) / dtSeconds;
            double alpha = dtSeconds / (RATE_FILTER_TIME_S + dtSeconds);
            filteredYawRate += alpha * (rate - filteredYawRate);
        } else {
            filteredYawRate = 0.0;
        }
        previousHeading = normalize360(headingDeg);
        headingSampleValid = true;
        // Raw rate too: the filter starts at 0 and would call a still-spinning robot settled.
        return firstSample || Math.abs(rate) > SETTLED_YAW_RATE_DPS
                || Math.abs(filteredYawRate) > SETTLED_YAW_RATE_DPS;
    }

    /** Call whenever manual driving ends, including idle, navigation and reset. */
    public void resetManualHeadingHold() {
        manualTargetValid = headingSampleValid = false;
        manualTargetHeading = previousHeading = filteredYawRate = 0.0;
        turnReleaseRemaining = manualHeadingError = manualCorrection = settleElapsed = 0.0;
        manualAssistStatus = "OFF";
    }

    public String getManualAssistStatus() { return manualAssistStatus; }
    public String getStatus() { return status; }
    public double getRequestedForward() { return fwd; }
    public double getRequestedRight() { return strafe; }
    public double getRequestedClockwise() { return turn; }
    public double getFrontLeftPower() { return frontLeftPower; }
    public double getFrontRightPower() { return frontRightPower; }
    public double getBackLeftPower() { return backLeftPower; }
    public double getBackRightPower() { return backRightPower; }
    public double getAppliedVoltageScale() { return appliedVoltageScale; }

    /** Linear robot-frame requests. Common scaling preserves wheel ratios. */
    public void drive(double fwd, double strafe, double turn, double voltageScale) {
        if (!finite(fwd) || !finite(strafe) || !finite(turn)
                || !finite(voltageScale) || voltageScale <= 0.0) {
            stop();
            status = "INVALID DRIVE INPUT";
            return;
        }
        double fl = fwd + strafe + turn;
        double fr = fwd - strafe - turn;
        double bl = fwd - strafe + turn;
        double br = fwd + strafe - turn;
        if (!finite(fl) || !finite(fr) || !finite(bl) || !finite(br)) {
            stop();
            status = "WHEEL MIX OVERFLOW";
            return;
        }
        this.fwd = fwd;
        this.strafe = strafe;
        this.turn = turn;
        appliedVoltageScale = Range.clip(voltageScale, MIN_VOLTAGE_SCALE, MAX_VOLTAGE_SCALE);
        double peak = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        double factor = appliedVoltageScale / Math.max(1.0, peak);
        double compensatedPeak = peak * factor;
        saturated = compensatedPeak > maxWheelPower;
        // Scale all four wheels together so the biggest is at most maxWheelPower (keeps the direction of travel).
        factor /= Math.max(1.0, compensatedPeak / maxWheelPower);
        frontLeftPower = Range.clip(fl * factor, -maxWheelPower, maxWheelPower);
        frontRightPower = Range.clip(fr * factor, -maxWheelPower, maxWheelPower);
        backLeftPower = Range.clip(bl * factor, -maxWheelPower, maxWheelPower);
        backRightPower = Range.clip(br * factor, -maxWheelPower, maxWheelPower);
        writePowers();
        status = fwd == 0.0 && strafe == 0.0 && turn == 0.0 ? "STOPPED" : "DRIVING";
    }

    /** Cap every wheel's |power| (0..1] for everything this drivetrain does from now on. Default 1.0. */
    public void setMaxWheelPower(double max) {
        if (!finite(max) || max <= 0.0) return;
        maxWheelPower = Math.min(1.0, max);
    }

    public void stop() {
        resetManualHeadingHold();
        fwd = strafe = turn = 0.0;
        frontLeftPower = frontRightPower = backLeftPower = backRightPower = 0.0;
        appliedVoltageScale = 1.0;
        saturated = false;
        status = "STOPPED";
        writePowers();
    }

    private void writePowers() {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    /** Optional diagnostics; MainTeleOP2 hides these in its normal display. */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Drive", status);
        telemetry.addData("Requested F / R / CW", "%.3f / %.3f / %.3f", fwd, strafe, turn);
        telemetry.addData("Front power L / R", "%.3f / %.3f", frontLeftPower, frontRightPower);
        telemetry.addData("Back power L / R", "%.3f / %.3f", backLeftPower, backRightPower);
        telemetry.addData("Drive voltage scale", "%.3f", appliedVoltageScale);
        telemetry.addData("Voltage boost limited", saturated);
        telemetry.addData("Manual hold error / CW correction", "%.2f deg / %.3f",
                manualHeadingError, manualCorrection);
    }

    private static double shape(double value) {
        if (!finite(value)) return Double.NaN;
        value = Range.clip(value, -1.0, 1.0);
        if (Math.abs(value) <= STICK_DEADBAND) return 0.0;
        double adjusted = (Math.abs(value) - STICK_DEADBAND) / (1.0 - STICK_DEADBAND);
        return Math.copySign(adjusted * adjusted * adjusted, value);
    }

    private static double wrapSignedDeg(double degrees) {
        double wrapped = degrees % 360.0;
        if (wrapped >= 180.0) wrapped -= 360.0;
        if (wrapped < -180.0) wrapped += 360.0;
        return wrapped;
    }

    private static double normalize360(double degrees) {
        double normalized = degrees % 360.0;
        if (normalized < 0.0) normalized += 360.0;
        // Avoid presenting -0.0 or 360.0 due to floating-point roundoff.
        return normalized == 0.0 || normalized >= 360.0 ? 0.0 : normalized;
    }

    private static boolean finite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
