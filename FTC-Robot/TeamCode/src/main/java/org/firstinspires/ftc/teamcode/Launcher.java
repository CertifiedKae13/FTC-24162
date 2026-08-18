package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Launcher wheel + angle servo. A starts, B stops, D-pad sets angle/speed. */
public class Launcher {
    private static final double LAUNCH_VEL_TOL = 50;

    private final DcMotorEx launchMotor;
    private final Servo launcherServo;

    private LaunchAngle launchAngle = LaunchAngle.RETRACTED;
    private boolean motorRunning = false;
    private double targetLaunchVel = -1500;   // default matches RETRACTED

    public Launcher(HardwareMap hardwareMap) {
        launchMotor = hardwareMap.get(DcMotorEx.class, "Launch");
        launchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcherServo = hardwareMap.get(Servo.class, "launcherServo");
        launcherServo.setDirection(Servo.Direction.REVERSE);
        launcherServo.setPosition(LaunchAngle.RETRACTED.servoPos);
    }

    public void update(Gamepad curr, Gamepad prev) {
        // ── D-pad : angle + target velocity ──
        if (GamepadUtil.rising(curr.dpad_down, prev.dpad_down)) {
            setLaunchAngle(LaunchAngle.RETRACTED);
            targetLaunchVel = -1500;
        }
        if (GamepadUtil.rising(curr.dpad_up, prev.dpad_up)) {
            setLaunchAngle(LaunchAngle.EXTENDED);
            targetLaunchVel = -2500;
        }

        // ── A = START ──
        if (GamepadUtil.rising(curr.a, prev.a)) {
            motorRunning = true;
        }

        // ── B = STOP (only if not used for intake reverse) ──
        if (GamepadUtil.rising(curr.b, prev.b) && !curr.right_bumper) {
            motorRunning = false;
            launchMotor.setVelocity(0);
        }

        // ── Keep motor running at the current target speed ──
        if (motorRunning) {
            launchMotor.setVelocity(targetLaunchVel);
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        double vel = launchMotor.getVelocity();
        boolean atTarget = motorRunning && Math.abs(vel - targetLaunchVel) < LAUNCH_VEL_TOL;
        telemetry.addLine("── Launcher ──");
        telemetry.addData(" Angle", launchAngle);
        telemetry.addData(" Motor", motorRunning ? "ON" : "off");
        telemetry.addData(" Vel", "%.0f / %.0f %s", vel, targetLaunchVel, atTarget ? "✓" : "");
    }

    private void setLaunchAngle(LaunchAngle angle) {
        launchAngle = angle;
        launcherServo.setPosition(angle.servoPos);
    }
}
