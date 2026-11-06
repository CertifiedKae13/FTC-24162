package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/** Individually checks motor mapping and physical forward direction. */
public final class MecanumMotorDirectionDebugger extends LinearOpMode {
    private static final double TEST_POWER = 0.20;

    @Override public void runOpMode() throws InterruptedException {
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        MecanumDrive.configureMotorDirections(frontLeft, frontRight, backLeft, backRight);
        DcMotorEx[] motors = {frontLeft, frontRight, backRight, backLeft};
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
        }
        String[] names = {"frontLeft (X)", "frontRight (Y)",
                "backRight (B)", "backLeft (A)"};
        telemetry.addLine("LIFT THE ROBOT so every wheel is clear of the floor.");
        telemetry.addLine("Hold LEFT BUMPER plus exactly one face button.");
        telemetry.addLine("X=frontLeft, Y=frontRight, B=backRight, A=backLeft.");
        telemetry.addLine("The selected wheel must spin in the robot's forward direction.");
        telemetry.update();
        waitForStart();
        try {
            while (opModeIsActive()) {
                boolean[] selected = {gamepad1.x, gamepad1.y, gamepad1.b, gamepad1.a};
                int count = 0, index = -1;
                for (int i = 0; i < selected.length; i++) {
                    if (selected[i]) { count++; index = i; }
                }
                boolean enabled = gamepad1.left_bumper && count == 1;
                for (int i = 0; i < motors.length; i++) {
                    motors[i].setPower(enabled && i == index ? TEST_POWER : 0);
                }
                telemetry.addData("Selected", enabled ? names[index] : "None: hold LB + one button");
                telemetry.addData("Power", enabled ? TEST_POWER : 0);
                telemetry.addLine("Release LB or button to stop. STOP ends the test.");
                telemetry.update();
                sleep(20);
            }
        } finally {
            for (DcMotorEx motor : motors) motor.setPower(0);
        }
    }
}
