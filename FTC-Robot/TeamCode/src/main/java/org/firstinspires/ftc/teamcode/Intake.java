package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/** Intake motor; toggled by D-pad left, reversed by B + right bumper. */
public class Intake {
    private static final double INTAKE_FWD_PWR = 1.0;
    private static final double INTAKE_REV_PWR = -0.5;

    private final DcMotor intakeMotor;
    private boolean running = false;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake");
    }

    public void update(Gamepad curr, Gamepad prev, double voltageScale) {
        if (GamepadUtil.rising(curr.dpad_left, prev.dpad_left)) {
            running = !running;
        }

        boolean reverseHeld = curr.b && curr.right_bumper;
        double raw;
        if (reverseHeld) raw = INTAKE_REV_PWR;
        else if (running) raw = INTAKE_FWD_PWR;
        else raw = 0;

        intakeMotor.setPower(Range.clip(raw * voltageScale, -1.0, 1.0));
    }
}
