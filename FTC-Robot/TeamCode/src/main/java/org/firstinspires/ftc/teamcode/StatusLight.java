package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Optional goBILDA RGB Indicator Light, wired as a servo called "statusLight" in the robot configuration. If it is
 * not configured this does nothing. The colour for each state is a servo position set in AutoConfig (tunable live):
 * check them against goBILDA's RGB indicator light colour chart.
 */
public final class StatusLight {
    public enum State { READY, MOVING, ABORTED, OFF }

    /** Name of the light's servo in the Robot Controller hardware configuration. */
    public static final String SERVO_NAME = "statusLight";

    private Servo servo;

    public StatusLight(HardwareMap hardwareMap) {
        try { servo = hardwareMap.tryGet(Servo.class, SERVO_NAME); } catch (RuntimeException e) { servo = null; }
    }

    public void set(State state) {
        if (servo == null) return;
        try {
            switch (state) {
                case READY: servo.setPosition(AutoConfig.LIGHT_READY); break;
                case MOVING: servo.setPosition(AutoConfig.LIGHT_MOVING); break;
                case ABORTED: servo.setPosition(AutoConfig.LIGHT_ABORTED); break;
                default: servo.setPosition(AutoConfig.LIGHT_OFF);
            }
        } catch (RuntimeException ignored) { }
    }
}
