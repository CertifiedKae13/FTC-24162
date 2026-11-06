package org.firstinspires.ftc.teamcode;

/**
 * Field-relative start pose, inches and degrees. The ONE place to set where the robot is placed.
 *
 * Field frame: origin at the CENTER of the field, heading 0 deg = facing +X, CCW-positive.
 * Which wall +X points toward is your choice: pick it once and use it for every start pose.
 *
 * Pinpoint is zeroed at reset, then set to this pose, so getX()/getY()/getHeadingDeg()
 * report field coordinates. Place the robot at this pose BEFORE init / reset.
 * Defaults (0, 0, 0) = robot starts at field center facing +X; change per match.
 */
public final class FieldStart {
    private FieldStart() {}

    public static final double X_IN = -63.0;   // RED start: robot center 9 in off the red wall (x = -72), touching it
    public static final double Y_IN = 0.0;     // TUNABLE: slide along the wall; keep out of the LOADING ZONE
    public static final double HEADING_DEG = 0.0;  // facing +X (toward blue / the hives); BLUE: X=+63, heading 180

    /** "Go home" target: field center, facing +X. Change here if home should be elsewhere. */
    public static final double HOME_X_IN = 0.0;
    public static final double HOME_Y_IN = 0.0;
    public static final double HOME_HEADING_DEG = 0.0;
}
