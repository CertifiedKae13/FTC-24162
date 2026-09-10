package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Mecanum drivetrain. Robot frame: +forward, +strafe RIGHT, +turn CLOCKWISE.
 * Assumes X rollers viewed from above. Motor directions are retained from the
 * supplied code, NOT physically verified. Positive logical wheel power must
 * propel that wheel forward. Odometry converts +left / +CCW at its drive call.
 */
public class Drivetrain2 {
    private static final double STICK_DEADBAND = 0.10;
    private static final double STRAFE_SCALE = 1.1;
    private static final double MIN_VOLTAGE_SCALE = 0.80;
    private static final double MAX_VOLTAGE_SCALE = 1.15;

    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private double fwd, strafe, turn;
    private double frontLeftPower, frontRightPower, backLeftPower, backRightPower;
    private double appliedVoltageScale = 1.0;
    private boolean saturated;
    private String status = "STOPPED";

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

    /** Driver input only: deadband, cubic shaping, and lateral adjustment. */
    public void update(Gamepad gamepad, double voltageScale) {
        if (gamepad == null) {
            stop();
            status = "INVALID GAMEPAD";
            return;
        }
        drive(shape(-gamepad.left_stick_y),
                shape(gamepad.left_stick_x) * STRAFE_SCALE,
                shape(gamepad.right_stick_x), voltageScale);
    }

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
        saturated = compensatedPeak > 1.0;
        factor /= Math.max(1.0, compensatedPeak);
        frontLeftPower = Range.clip(fl * factor, -1.0, 1.0);
        frontRightPower = Range.clip(fr * factor, -1.0, 1.0);
        backLeftPower = Range.clip(bl * factor, -1.0, 1.0);
        backRightPower = Range.clip(br * factor, -1.0, 1.0);
        writePowers();
        status = fwd == 0.0 && strafe == 0.0 && turn == 0.0 ? "STOPPED" : "DRIVING";
    }

    public void stop() {
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

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Drive", status);
        telemetry.addData("Requested F / R / CW", "%.3f / %.3f / %.3f", fwd, strafe, turn);
        telemetry.addData("Front power L / R", "%.3f / %.3f", frontLeftPower, frontRightPower);
        telemetry.addData("Back power L / R", "%.3f / %.3f", backLeftPower, backRightPower);
        telemetry.addData("Drive voltage scale", "%.3f", appliedVoltageScale);
        telemetry.addData("Voltage boost limited", saturated);
    }

    private static double shape(double value) {
        if (!finite(value)) return Double.NaN;
        value = Range.clip(value, -1.0, 1.0);
        if (Math.abs(value) <= STICK_DEADBAND) return 0.0;
        double adjusted = (Math.abs(value) - STICK_DEADBAND) / (1.0 - STICK_DEADBAND);
        return Math.copySign(adjusted * adjusted * adjusted, value);
    }

    private static boolean finite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
