package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Sets "launchServo" to position 0 and spins "launchMotor" at -1300 RPM.
 * Press X to toggle the motor on/off (starts off).
 * Hold B to move the servo up (toward 1), hold A to move it down (toward 0).
 * Ticks per revolution come from the motor type chosen in the hardware config,
 * so the RPM is correct for whichever motor is configured.
 * STOP the opmode to stop the motor.
 */
@TeleOp(name = "LaunchTester", group = "Test")
public class LaunchTester extends LinearOpMode {
    private static final double TARGET_RPM = 900;
    private static final double SERVO_START_POSITION = 0;
    private static final double SERVO_RANGE_DEGREES = 180; // full 0..1 travel; change to match your servo
    private static final double RPM_SMOOTHING = 0.2;    // 0..1, lower = smoother but slower to react
    private static final double AT_SPEED_TOLERANCE = 0.03; // within 3% of target counts as at speed
    private static final double SERVO_SPEED_PER_SEC = 0.5; // full travel in 2 seconds

    @Override public void runOpMode() throws InterruptedException {
        Servo launchServo = hardwareMap.get(Servo.class, "launchServo");
        DcMotorEx launchMotor = hardwareMap.get(DcMotorEx.class, "launchMotor");

        // Read all encoders from the hub in one I2C-free bulk read per loop.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        launchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchServo.setPosition(SERVO_START_POSITION);
        double servoTarget = SERVO_START_POSITION;

        double ticksPerRev = launchMotor.getMotorType().getTicksPerRev();
        double targetTicksPerSec = TARGET_RPM * ticksPerRev / 60.0;

        telemetry.addData("Ticks per rev", "%.1f", ticksPerRev);
        telemetry.addLine("Press START, then X toggles launchMotor. B/A move servo.");
        telemetry.update();
        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        double smoothedRpm = 0;
        boolean motorOn = false;
        boolean previousX = false;
        try {
            while (opModeIsActive()) {
                for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.clearBulkCache();
                double dt = loopTimer.seconds();
                loopTimer.reset();
                if (gamepad1.b) servoTarget += SERVO_SPEED_PER_SEC * dt;
                if (gamepad1.a) servoTarget -= SERVO_SPEED_PER_SEC * dt;
                servoTarget = Range.clip(servoTarget, 0.0, 1.0);
                launchServo.setPosition(servoTarget);
                if (gamepad1.x && !previousX) motorOn = !motorOn;
                previousX = gamepad1.x;
                launchMotor.setVelocity(motorOn ? targetTicksPerSec : 0);
                double actualRpm = launchMotor.getVelocity() * 60.0 / ticksPerRev;
                smoothedRpm += RPM_SMOOTHING * (actualRpm - smoothedRpm);
                double targetRpm = motorOn ? TARGET_RPM : 0;
                boolean atSpeed = motorOn
                        && Math.abs(smoothedRpm - targetRpm) <= Math.abs(TARGET_RPM) * AT_SPEED_TOLERANCE;
                telemetry.addData("Servo position (B up / A down)", "%.2f", servoTarget);
                telemetry.addData("Servo angle", "%.1f deg", servoTarget * SERVO_RANGE_DEGREES);
                telemetry.addData("Motor (X toggles)", motorOn ? "ON" : "OFF");
                telemetry.addData("Target RPM", "%.0f", motorOn ? TARGET_RPM : 0);
                telemetry.addData("Actual RPM (smoothed)", "%.0f", smoothedRpm);
                telemetry.addData("Actual RPM (raw)", "%.0f", actualRpm);
                telemetry.addData("At speed", atSpeed ? "YES" : "no");
                telemetry.addData("Encoder ticks", launchMotor.getCurrentPosition());
                telemetry.addData("Motor power", "%.2f", launchMotor.getPower());
                telemetry.update();
            }
        } finally {
            launchMotor.setVelocity(0);
            launchMotor.setPower(0);
        }
    }
}
