package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Very simple drive test: forward/backward and strafe only. No odometry, IMU, or heading hold.
 *
 * Controls (gamepad1):
 *   D-pad up / down, or left stick Y   -> forward / backward
 *   D-pad left / right, or left stick X -> strafe left / right
 *   Bumper (either) pressed             -> toggle slow (0.25) / normal (0.5) speed
 *   No turning, on purpose.
 *
 * Expected motion: up = all four wheels push the robot forward; right = FL and BR forward,
 * FR and BL backward (robot slides right).
 *
 * Motor directions: left FORWARD, right REVERSE, matching Drivetrain2 and
 * MecanumDrive.configureMotorDirections. NOTE: Drivetrain.java uses the OPPOSITE
 * (left REVERSE, right FORWARD). Confirm on the real robot (lift it first) and flip
 * LEFT_DIRECTION / RIGHT_DIRECTION below if the wheels push the wrong way.
 */
@TeleOp(name = "SimpleDriveTest", group = "Test")
public class SimpleDriveTest extends OpMode {
    private static final DcMotor.Direction LEFT_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction RIGHT_DIRECTION = DcMotor.Direction.REVERSE;
    private static final double SPEED = 0.4;       // used until a bumper is pressed
    private static final double SLOW_SPEED = 0.25;
    private static final double NORMAL_SPEED = 0.5;
    private static final double STICK_DEADBAND = 0.10;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private double speed = SPEED;
    private boolean bumperWasDown;
    private double fwd, strafe, fl, fr, bl, br;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(LEFT_DIRECTION);
        backLeft.setDirection(LEFT_DIRECTION);
        frontRight.setDirection(RIGHT_DIRECTION);
        backRight.setDirection(RIGHT_DIRECTION);
        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        stopMotors();
        telemetry.addLine("SimpleDriveTest: LIFT THE ROBOT first.");
        telemetry.addData("Directions", "left %s, right %s", LEFT_DIRECTION, RIGHT_DIRECTION);
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean bumper = gamepad1.left_bumper || gamepad1.right_bumper;
        if (bumper && !bumperWasDown) speed = speed == SLOW_SPEED ? NORMAL_SPEED : SLOW_SPEED;
        bumperWasDown = bumper;

        // FTC stick Y is negative when pushed forward.
        fwd = deadband(-gamepad1.left_stick_y);
        strafe = deadband(gamepad1.left_stick_x);
        if (gamepad1.dpad_up) fwd = 1.0;
        if (gamepad1.dpad_down) fwd = -1.0;
        if (gamepad1.dpad_right) strafe = 1.0;   // +strafe = RIGHT
        if (gamepad1.dpad_left) strafe = -1.0;

        // Mecanum mix without turning.
        fl = fwd + strafe;
        fr = fwd - strafe;
        bl = fwd - strafe;
        br = fwd + strafe;
        double peak = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        fl = fl / peak * speed;
        fr = fr / peak * speed;
        bl = bl / peak * speed;
        br = br / peak * speed;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

        telemetry.addData("Speed cap", "%.2f (bumper toggles 0.25 / 0.50)", speed);
        telemetry.addData("Command fwd / strafe(+right)", "%.2f / %.2f", fwd, strafe);
        telemetry.addData("Power FL / FR", "%.2f / %.2f", fl, fr);
        telemetry.addData("Power BL / BR", "%.2f / %.2f", bl, br);
        telemetry.addData("Directions", "left %s, right %s", LEFT_DIRECTION, RIGHT_DIRECTION);
        telemetry.update();
    }

    @Override
    public void stop() {
        stopMotors();
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private static double deadband(double value) {
        return Math.abs(value) <= STICK_DEADBAND ? 0.0 : Range.clip(value, -1.0, 1.0);
    }
}
