package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/**
 * Live-tunable values for MainAuto2. Open the FTC Dashboard (http://192.168.43.1:8080/dash) > Configuration >
 * AutoConfig, change a number, and the next move uses it: no redeploy. The defaults below are exactly the
 * constants MainAuto2 used before they moved here. Values reset to these defaults when the app restarts.
 */
@Config
public class AutoConfig {
    // Robot size (inches)
    public static double ROBOT_RADIUS_IN = 9.0;        // half the robot's widest dimension (keep-out inflation)
    public static double ROBOT_HALF_LENGTH_IN = 9.0;   // center-to-front/back; sets the wall offset
    public static double ROBOT_HALF_WIDTH_IN = 9.0;    // center-to-side, for wall bumps
    // Routing around the hive
    public static double SAFETY_MARGIN_IN = 3.0;       // extra gap beyond the robot
    public static double WAYPOINT_PUSH = 1.15;         // detour waypoints sit this far past the zone edge
    // Wall bump
    public static double BUMP_POWER = 0.3;             // gentle, you are driving into a wall
    public static double BUMP_OVERDRIVE_IN = 4.0;      // aim this far past the contact point
    public static double BUMP_MAX_SKEW_DEG = 20.0;     // heading must be this close to square to the wall
    public static double BUMP_REACH_TOL_IN = 12.0;     // stopped farther than this from the wall = stuck elsewhere
    // Start pose selector and timing
    public static double START_Y_MAX_IN = 50.0;        // keep start y clear of walls/loading zone
    public static double MOVE_TIMEOUT_S = 12.0;        // give up on a single drive/turn
    public static double CALIBRATION_TIMEOUT_S = 8.0;  // wait this long for the Pinpoint to be ready
    // Battery compensation
    public static double NOMINAL_VOLTAGE = 12.0;       // battery compensation reference
    public static double MIN_VOLTAGE_SCALE = 0.80;
    public static double MAX_VOLTAGE_SCALE = 1.15;
    // Panels-style extras
    public static boolean DARK_MAP = false;            // use the dark BIOBUZZ field image on the Dashboard
    public static boolean CAPTURE_ENABLED = true;      // record each run to /sdcard/FIRST/capture/*.csv
    public static double CAPTURE_HZ = 20.0;            // capture rows per second
    public static double CAMERA_FPS = 10.0;            // Dashboard camera stream rate (black screen if no camera)
    public static boolean SHOW_GAMEPAD = true;         // draw the on-screen controller on the field view
    // goBILDA RGB indicator light servo positions (unverified defaults: check goBILDA's colour chart and tune live)
    public static double LIGHT_READY = 0.500;          // green
    public static double LIGHT_MOVING = 0.611;         // blue
    public static double LIGHT_ABORTED = 0.277;        // red
    public static double LIGHT_OFF = 0.0;
    // Limelight AprilTag vision (LimelightVision)
    public static double LL_PIPELINE = 0;              // slot of your AprilTag pipeline on the Limelight
    public static double LL_POLL_HZ = 100;             // how often to ask the Limelight for data
    public static double LL_MAX_STALENESS_MS = 100;    // older than this = "stale", treated as no valid target
    public static double LL_TILT_AXIS = 0;             // which tag angle is the hive tilt: 0 pitch, 1 yaw, 2 roll (calibrate)
    public static double LL_TILT_SIGN = 1.0;           // flip to -1 if the hive tilt reads backwards
    public static double LL_TILT_ZERO_DEG = 0.0;       // tag pitch (deg) when the hive is in its reference state
    public static double LL_FIELD_ROTATION_DEG = 0.0;  // rotate the Limelight map frame into ours (+X toward blue)
    // Limelight distance WITHOUT AprilTags (from ty). FILL IN: measured on your robot. NaN = not set (distance = NaN).
    public static double LL_CAMERA_HEIGHT_IN = Double.NaN;   // lens to floor
    public static double LL_CAMERA_DOWN_DEG = Double.NaN;    // camera tilted down from horizontal (+ = down)
    public static double LL_TARGET_HEIGHT_IN = 1.4;          // target centre above floor (pollen ball ~2.8 in -> 1.4)
    // A move that ends this close to its target counts as arrived (instead of aborting the routine)
    public static double ARRIVE_TOLERANCE_IN = 1.0;
    public static double ARRIVE_TOLERANCE_DEG = 3.0;
    // Turns: friction push so the robot keeps rotating in the last few degrees (TeleOp uses its own 0.025 / 8 deg)
    public static double TURN_STATIC_POWER = 0.10;     // measure your breakaway turn power and set this a bit above it
    public static double TURN_STATIC_TAPER_DEG = 3.0;  // full push above this error, tapering to 0 at the target
    public static double TURN_RETRIES = 2;             // re-issue a turn that stalls short this many times
    // Decision speed: multiplies the controller's settle waits before it declares a move done (1.0 = old)
    public static double SETTLE_SCALE = 0.5;
    // Pause at the end of a move: within ARRIVE_TOLERANCE of the target, a move that stops creeping is
    // finished after this long (was 1.25 s for drives, 1.0 s for turns)
    public static double FINISH_PAUSE_S = 0.1;
    // Flower check with the Limelight (FlowerAuto): a colour pipeline tuned to the flowers' green posts
    public static double LL_FLOWER_PIPELINE = 1;        // Limelight slot of the green flower pipeline
    public static double LL_FLOWER_LOOK_S = 0.6;        // how long to look after facing a flower
    public static double LL_FLOWER_ALIGN_DEG = 2.0;     // if the flower is more than this off-centre, turn to centre it
    public static double LL_STEADY_FRAMES = 5;          // flower check: median of this many fresh frames
    public static double LL_STEADY_MIN_FRAMES = 3;      // fewer good frames than this in LL_FLOWER_LOOK_S = not seen
    // Flower localization (no AprilTags): absolute x/y from a flower's bearing + post spacing (FlowerFix)
    public static double LL_FLOWER_RANGE_K = 2790;      // depth_in x spacing_px head-on. ESTIMATE: run "Flower Range Cal"
    public static double LL_CAMERA_FORWARD_IN = 3.0;    // Limelight lens ahead of robot centre. MEASURE
    public static double LL_CAMERA_LEFT_IN = 0.0;       // Limelight lens left (+) / right (-) of robot centre. MEASURE
    public static double LL_FIX_APPLY = 0;              // 1 = correct odometry with flower fixes; 0 = report only
    public static double LL_FIX_GAIN = 0.7;             // fraction of the correction applied per fix
    public static double LL_FIX_MAX_JUMP_IN = 6.0;      // fixes farther than this from odometry are rejected
    public static double LL_FIX_MAX_RANGE_IN = 60.0;    // beyond this, 1 px of spacing error is several inches
    public static double LL_FIX_MAX_VIEW_DEG = 45.0;    // more oblique than this = unreliable spacing
}
