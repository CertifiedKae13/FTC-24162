package org.firstinspires.ftc.teamcode;

/** Small shared helpers for reading controller state. */
public final class GamepadUtil {
    private GamepadUtil() {}

    /** True when a button transitions from released to pressed. */
    public static boolean rising(boolean now, boolean before) {
        return now && !before;
    }
}
