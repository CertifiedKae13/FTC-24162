package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

/**
 * Spins drive wheels individually or in groups, only while buttons are held.
 *
 * Single wheels (hold any combination to run several at once):
 *   X = frontLeft, Y = frontRight, A = backLeft, B = backRight
 * Pair shortcuts:
 *   D-pad left = both left wheels, D-pad right = both right wheels,
 *   D-pad up = both front wheels, D-pad down = both back wheels
 * Speed: velocity-controlled (RPM), so every wheel is asked for the same RPM.
 *   Left bumper = 50 RPM lower, right bumper = 50 RPM higher.
 * Hold the right trigger to spin the selected wheels in reverse.
 *
 * Directions come from MecanumDrive.configureMotorDirections, so "forward"
 * here means the wheel pushes the robot forward. LIFT THE ROBOT first.
 */
@TeleOp(name = "WheelTester", group = "Test")
public class WheelTester extends LinearOpMode {
    private static final double START_RPM = 300;
    private static final double RPM_STEP = 50;
    private static final double MIN_RPM = 50;

    @Override public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        MecanumDrive.configureMotorDirections(frontLeft, frontRight, backLeft, backRight);
        DcMotorEx[] motors = {frontLeft, frontRight, backLeft, backRight};
        String[] names = {"frontLeft", "frontRight", "backLeft", "backRight"};
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setVelocity(0);
        }

        // Highest RPM every motor can actually reach, so all wheels can match the target.
        double maxRpm = Double.MAX_VALUE;
        for (DcMotorEx motor : motors) {
            maxRpm = Math.min(maxRpm, motor.getMotorType().getMaxRPM()
                    * motor.getMotorType().getAchieveableMaxRPMFraction());
        }
        double targetRpm = Math.min(START_RPM, maxRpm);
        boolean previousLb = false, previousRb = false;
        telemetry.addLine("LIFT THE ROBOT so every wheel is clear of the floor.");
        telemetry.addLine("X=frontLeft Y=frontRight A=backLeft B=backRight (hold several to run several)");
        telemetry.addLine("D-pad: left/right/up/down = both left/right/front/back wheels");
        telemetry.addLine("Bumpers: RPM down/up. Right trigger: reverse.");
        telemetry.update();
        waitForStart();
        try {
            while (opModeIsActive()) {
                if (gamepad1.right_bumper && !previousRb) targetRpm += RPM_STEP;
                if (gamepad1.left_bumper && !previousLb) targetRpm -= RPM_STEP;
                previousRb = gamepad1.right_bumper;
                previousLb = gamepad1.left_bumper;
                targetRpm = Range.clip(targetRpm, MIN_RPM, maxRpm);

                boolean[] selected = {
                        gamepad1.x || gamepad1.dpad_left || gamepad1.dpad_up,    // frontLeft
                        gamepad1.y || gamepad1.dpad_right || gamepad1.dpad_up,   // frontRight
                        gamepad1.a || gamepad1.dpad_left || gamepad1.dpad_down,  // backLeft
                        gamepad1.b || gamepad1.dpad_right || gamepad1.dpad_down  // backRight
                };
                double signedRpm = gamepad1.right_trigger > 0.5 ? -targetRpm : targetRpm;
                StringBuilder running = new StringBuilder();
                for (int i = 0; i < motors.length; i++) {
                    // Convert RPM to ticks/s with this motor's own encoder resolution.
                    double ticksPerSec = signedRpm * motors[i].getMotorType().getTicksPerRev() / 60.0;
                    motors[i].setVelocity(selected[i] ? ticksPerSec : 0);
                    if (selected[i]) running.append(running.length() > 0 ? " + " : "").append(names[i]);
                }
                telemetry.addData("Running", running.length() > 0 ? running : "none");
                telemetry.addData("Target speed", "%.0f RPM %s (max reachable %.0f)", targetRpm,
                        gamepad1.right_trigger > 0.5 ? "(REVERSE)" : "(forward)", maxRpm);
                for (int i = 0; i < motors.length; i++) {
                    double ticksPerRev = motors[i].getMotorType().getTicksPerRev();
                    double velocity = motors[i].getVelocity(); // ticks per second
                    telemetry.addData(names[i], "%d ticks | %.0f ticks/s | %.0f RPM (%.1f ticks/rev)",
                            motors[i].getCurrentPosition(), velocity,
                            ticksPerRev > 0 ? velocity * 60.0 / ticksPerRev : 0, ticksPerRev);
                }
                telemetry.addLine("Release buttons to stop. STOP ends the test.");
                telemetry.update();
                sleep(20);
            }
        } finally {
            for (DcMotorEx motor : motors) {
                motor.setVelocity(0);
                motor.setPower(0);
            }
        }
    }
}
