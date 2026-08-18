package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Putter servo; fires on stick click and auto-returns after a hold time. */
public class Putter {
    private static final double PUTTER_FIRE_POS = 0.7;
    private static final double PUTTER_HOME_POS = 0.00;
    private static final long PUTTER_HOLD_MS = 500;

    private final Servo putterServo;
    private final ElapsedTime putterTimer = new ElapsedTime();
    private boolean firing = false;

    public Putter(HardwareMap hardwareMap) {
        putterServo = hardwareMap.get(Servo.class, "putterServo");
        putterServo.setPosition(PUTTER_HOME_POS);
    }

    public void update(Gamepad curr, Gamepad prev) {
        boolean fire = GamepadUtil.rising(curr.left_stick_button, prev.left_stick_button)
                   || GamepadUtil.rising(curr.right_stick_button, prev.right_stick_button);

        if (fire) {
            putterServo.setPosition(PUTTER_FIRE_POS);
            putterTimer.reset();
            firing = true;
        }

        if (firing && putterTimer.milliseconds() > PUTTER_HOLD_MS) {
            putterServo.setPosition(PUTTER_HOME_POS);
            firing = false;
        }
    }

    /** True when the putter is retracted / safe for the plate to rotate. */
    public boolean isClear() {
        if (!firing) return true;
        return putterTimer.milliseconds() > PUTTER_HOLD_MS;
    }
}
