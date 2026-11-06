package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/**
 * Shooter aiming settings (ShotSolver / Shooter). Live-tunable in FTC Dashboard.
 * Every value marked MEASURE or CAD is a placeholder: the shot is only as good as these numbers.
 * Field frame as FieldStart: inches, origin at the centre, +X toward blue, +Y away from the audience, CCW heading.
 */
@Config
public class ShotConfig {
    // ===== Hive model (Competition Manual 9.6; confirm with the official field CAD) =====
    public static double HIVE_PIVOT_HEIGHT_IN = 43.95;   // pivot axis above the tiles (manual)
    public static double RED_HIVE_X_IN = -12.75;         // hive centres 25.5 in apart along X (manual Fig 9-10)
    public static double BLUE_HIVE_X_IN = 12.75;
    public static double CELL_ALONG_IN = 15.44;          // cell centre from hive centre along the hive (CAD: 18.84 gap + 12.04 cell, est.)
    public static double CELL_AIM_ABOVE_PIVOT_IN = 15.6; // cell OPENING centre above the pivot with the hive level (CAD, est.)
    public static double TAG_TO_AIM_UP_IN = 8.0;         // tag cluster (cell underside) up to the opening centre (CAD, GUESS)
    /** Which end of the hive is +Y. ASSUMPTION: audience cells (tags 34-37, 38-41) at -Y, scoring cells at +Y. */
    public static double SCORING_CELL_SIDE = 1;          // +1: scoring cell at +Y; -1 if it is at -Y
    /** Target cell: 1 = scoring cell, -1 = audience cell, 0 = whichever is nearer the robot. */
    public static double TARGET_CELL = 0;
    // Hive tilt (+ = the +Y end of the hive UP). Comes from LimelightVision (calibrate LL_TILT_* in AutoConfig).
    public static double TILT_HOLD_S = 5.0;              // reuse the last tag tilt this long after the tags are lost
    public static double TILT_DEFAULT_DEG = 0.0;         // assumed tilt when it was never / not recently measured

    // ===== Launcher on the robot (MEASURE) =====
    public static double LAUNCHER_FWD_IN = 0.0;          // ball exit ahead of robot centre
    public static double LAUNCHER_LEFT_IN = 0.0;         // ball exit left of robot centre
    public static double LAUNCHER_HEIGHT_IN = 12.0;      // ball exit above the tiles
    public static double FLYWHEEL_RADIUS_IN = 2.0;          // MEASURE
    public static double SPEED_EFFICIENCY = 0.5;         // ball exit speed / wheel surface speed (MEASURE: 0.3-0.7 typical)
    /** Flywheel RPM by distance: {distance in, rpm}. Ascending distance. FILL IN from test shots. */
    public static double[][] RPM_TABLE = {{24, 2600}, {48, 2800}, {72, 3100}, {96, 3400}};   // PLACEHOLDER
    public static boolean HIGH_ARC = true;               // drop into the cell steeply (false = flat, fast shot)

    // ===== Hood servo (launchServo) angle mapping (MEASURE with a phone level at two positions) =====
    public static double HOOD_POS_A = 0.0, HOOD_ANGLE_A_DEG = 20.0;
    public static double HOOD_POS_B = 1.0, HOOD_ANGLE_B_DEG = 70.0;
    public static double HOOD_MIN_DEG = 20.0, HOOD_MAX_DEG = 70.0;

    // ===== Fusing the two position estimates =====
    public static double ODO_SIGMA_IN = 2.0;             // odometry error at the hive (grows if the tilt is only assumed)
    public static double TILT_ASSUMED_SIGMA_IN = 4.0;    // extra error when the tilt is assumed, not measured
    public static double TAG_SIGMA_BASE_IN = 0.5;        // tag error up close ...
    public static double TAG_SIGMA_PER_IN = 0.02;        // ... plus this much per inch of range
    public static double FUSE_GATE_IN = 8.0;             // estimates farther apart than this = one is wrong: don't average
}
