package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * PINEAPPLE AUTO: autonomous version of MainTeleOP2 (left untouched).
 * Same hardware (Drivetrain2 motors, Pinpoint via Odometry2) and same navigation controller.
 * TeleOp 2 has no arm/intake/servos, so there are no mechanism steps to convert.
 *
 * All robot motion goes through drive() and turn() (plus bump(), a gentle wall push that corrects the
 * pose). Edit runSequence() to change the routine.
 * Field frame: origin = field center, heading 0 = +X, CCW positive (see FieldStart).
 * Place the robot at the FieldStart pose and keep it still during INIT.
 */
@Autonomous(name = "Pineapple2 Auto", group = "Competition")
public class MainAuto2 extends LinearOpMode {
    // ===== TUNABLE: loop around the field =====
    private static final double LOOP_INSET_IN = 20.0;   // TUNABLE: loop points sit this far in from each corner
    private static final double UPPER_RIGHT_NUDGE_IN = 3.0; // TUNABLE: pull the upper-right (3rd) point in from both walls
    private static final double TURN_POWER = 1.0;       // TUNABLE: turn speed (was 0.5: doubled)
    private static final double LOOP_POWER = 1.0;       // TUNABLE: speed around the loop (was 0.5: doubled)
    /** Circle auto only: no wheel ever gets more than this power (stops the robot tipping back on hard stops). */
    private static final double MAX_AUTO_SPEED = 0.5;
    private static final double START_FORWARD_IN = 24.0; // TUNABLE: straight-ahead move off the wall before the loop
    private static final double FIELD_LIMIT_IN = FieldMap.HALF - 9.0; // TUNABLE: stay this far inside walls
    private static final double WALL_IN = FieldMap.HALF;

    // ===== TUNABLE: sequence values (edit runSequence below) =====
    // ===== TUNABLE: behaviour =====

    private Drivetrain2 drivetrain;
    private StatusLight light;
    private LimelightVision limelight;                 // AprilTag tilt + pose; "not configured" if no Limelight
    private CaptureLog capture;                       // started when the run begins; inert if off or unwritable
    private String cameraStatus = "not started";
    private Odometry2 odometry;
    private VoltageSensor voltageSensor;
    private final ElapsedTime clock = new ElapsedTime();

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        }
        drivetrain = new Drivetrain2(hardwareMap);
        // Only the circle auto itself: Flower Auton extends this class and keeps full power.
        if (getClass() == MainAuto2.class) drivetrain.setMaxWheelPower(MAX_AUTO_SPEED);
        drivetrain.stop();
        odometry = new Odometry2(hardwareMap, drivetrain);
        odometry.init();
        light = new StatusLight(hardwareMap);                // no-op unless a "statusLight" servo is configured
        light.set(StatusLight.State.READY);
        limelight = new LimelightVision(hardwareMap);
        cameraStatus = CameraFeed.start(hardwareMap);        // Dashboard camera view: the camera, or a black screen if none

        // INIT start-pose selector: the robot must touch its own alliance wall (rule G304), so
        // only the position ALONG the wall is unknown. Driver picks it here; wall + heading follow.
        boolean red = true;
        double startY = FieldStart.Y_IN;
        boolean up = false, down = false, left = false, right = false, lb = false, rb = false;
        boolean dirty = true;
        while (!isStarted() && !isStopRequested()) {
            odometry.update();
            odometry.updateReset();
            drivetrain.stop();
            if (gamepad1.dpad_left && !left) { red = true; dirty = true; }
            if (gamepad1.dpad_right && !right) { red = false; dirty = true; }
            if (gamepad1.dpad_up && !up) { startY += 1; dirty = true; }
            if (gamepad1.dpad_down && !down) { startY -= 1; dirty = true; }
            if (gamepad1.right_bumper && !rb) { startY += 6; dirty = true; }
            if (gamepad1.left_bumper && !lb) { startY -= 6; dirty = true; }
            left = gamepad1.dpad_left; right = gamepad1.dpad_right; up = gamepad1.dpad_up;
            down = gamepad1.dpad_down; lb = gamepad1.left_bumper; rb = gamepad1.right_bumper;
            startY = Range.clip(startY, -AutoConfig.START_Y_MAX_IN, AutoConfig.START_Y_MAX_IN);
            double startX = (red ? -1 : 1) * (WALL_IN - AutoConfig.ROBOT_HALF_LENGTH_IN);
            double startH = red ? 0.0 : 180.0;   // face the middle
            if (dirty) { odometry.setFieldPose(startX, startY, startH); dirty = false; }
            telemetry.addData("Start pose", "%s  x %.0f  y %.0f  h %.0f", red ? "RED" : "BLUE",
                    startX, startY, startH);
            telemetry.addLine("D-pad L/R = alliance, U/D = y +-1 in, bumpers = y +-6 in");
            telemetry.addData("Status", "INIT / KEEP STILL");
            telemetry.addData("Camera", cameraStatus);
            limelight.setAlliance(red);
            LimelightVision.Reading ll = limelight.update(odometry.getHeadingDeg());
            PoseDisplay.setVision(ll);
            telemetry.addData("Limelight", ll.valid
                    ? String.format("tilt %s  pose %s", ll.hasHiveTilt ? String.format("%.1f deg (%s hive)", ll.hiveTiltDeg,
                            ll.hiveIsRed ? "RED" : "BLUE") : "-",
                            ll.hasPose ? String.format("(%.1f, %.1f) h %.0f", ll.x, ll.y, ll.headingDeg) : "-")
                    : ll.status);
            telemetry.addData("Pinpoint", "%s / %s", odometry.getStatus(),
                    odometry.isReady() ? "READY" : "NOT READY");
            telemetry.update();
        }
        if (isStopRequested()) {
            light.set(StatusLight.State.OFF);
            CameraFeed.stop();
            limelight.close();
            return;
        }
        capture = CaptureLog.start();

        try {
            if (waitForOdometry()) runSequence();
            else telemetry.addData("Status", "ODOMETRY NOT READY - AUTO ABORTED");
            while (aborted && opModeIsActive()) {   // keep the reason on screen until STOP
                drivetrain.stop();
                telemetry.addData("ROUTINE ABORTED", abortReason);
                telemetry.addData("Limelight pose", limelightText(limelight.update(odometry.getHeadingDeg())));
                telemetry.addData("Est", "x %.1f  y %.1f  h %.0f", odometry.getX(),
                        odometry.getY(), odometry.getHeadingDeg());
                telemetry.update();
                odometry.update();
            }
        } finally {
            light.set(StatusLight.State.OFF);
            if (capture != null) capture.close();
            limelight.close();
            CameraFeed.stop();
            odometry.cancelNavigation();
            drivetrain.stop();
            telemetry.update();
        }
    }

    /**
     * THE ROUTINE: edit only this method. Placeholder path; every value is TUNABLE.
     *   drive(power, inches)  - straight along current heading (negative inches = backward)
     *   drive(power, x, y)    - go to field coordinate (inches), keeping current heading
     *   turn(degrees)         - rotate in place, CCW positive
     * power is 0.1 to 1.0 (scales max drive/turn speed).
     */
    void runSequence() {
        // Loop around the field through 4 points, each 20 in in from a corner (x and y): a square at +-52 in.
        // Counter-clockwise from the red/audience corner; repeats until STOP (or a move fails).
        double c = FieldMap.HALF - LOOP_INSET_IN;        // 72 - 20 = 52
        double u = c - UPPER_RIGHT_NUDGE_IN;            // 3rd point (upper right) sits a little further from the walls
        double[][] loop = {{-c, -c}, {c, -c}, {u, u}, {-c, c}};
        // Leave the wall FORWARD first. Going straight from the wall start (-63, 0) to (-52, -52) is almost all
        // sideways (11 in forward, 52 in sideways) and runs along the wall right past the flower at (-72, -23).
        drive(LOOP_POWER, START_FORWARD_IN);
        while (opModeIsActive() && !aborted) {
            for (double[] p : loop) {
                if (!opModeIsActive() || aborted) return;
                faceAndDrive(LOOP_POWER, p[0], p[1]);    // turn to face the next corner, then drive straight at it
            }
        }
    }

    /**
     * Turn toward (x, y) and then drive there, so every leg is a FORWARD move. The one move that worked was a
     * forward one; the loop's next leg was almost pure sideways (strafe), which on this robot ends in the
     * controller's "no position progress" stop and aborted the routine.
     */
    void faceAndDrive(double power, double x, double y) {
        if (!opModeIsActive() || aborted) return;
        double dx = x - odometry.getX(), dy = y - odometry.getY();
        if (Math.hypot(dx, dy) < 1.0) return;
        double bearing = Math.toDegrees(Math.atan2(dy, dx));
        double delta = wrap180(bearing - odometry.getHeadingDeg());
        // Face the leg either nose-first or tail-first, whichever is the smaller turn. On the loop this makes the
        // corner turns alternate (+90, -90, ...) instead of all going the same way, so the robot's total rotation
        // stays near zero and the heading error that builds up with every same-direction turn cannot accumulate
        // lap after lap (that growing error is what rotated the loop a bit more each lap into the flower).
        if (Math.abs(delta) > 90) delta = wrap180(delta + 180);
        if (Math.abs(delta) > 2.0) turn(delta);
        drive(power, x, y);
    }

    /** Turn in place to an absolute field heading (degrees). */
    void turnTo(double headingDeg) {
        double delta = wrap180(headingDeg - odometry.getHeadingDeg());
        if (Math.abs(delta) > 1.0) turn(delta);
    }

    static double wrap180(double d) { return ((d % 360) + 540) % 360 - 180; }

    // ===== Movement helpers: the ONLY code that moves the robot =====

    /** Drive straight `distance` inches along the current heading. Negative power or distance reverses. */
    void drive(double power, double distance) {
        if (!opModeIsActive() || aborted) return;
        double signed = Math.signum(power == 0 ? 1 : power) * distance;
        double h = Math.toRadians(odometry.getHeadingDeg());
        double x = odometry.getX() + signed * Math.cos(h);
        double y = odometry.getY() + signed * Math.sin(h);
        driveRouted(power, x, y);
    }

    /** Drive to field coordinate (x, y) in inches, holding the current heading. */
    void drive(double power, double x, double y) {
        if (!opModeIsActive() || aborted) return;
        driveRouted(power, x, y);
    }

    /** Rotate in place by `degrees` (CCW positive, field frame). */
    void turn(double degrees) {
        if (!opModeIsActive() || aborted) return;
        turnCount++;
        boolean first = turnCount == 1;
        double x = odometry.getX(), y = odometry.getY();
        double target = odometry.getHeadingDeg() + degrees;
        // Only the FIRST turn keeps the micro adjustments (creep to 0.5 deg, retry a stalled turn). Every later turn
        // is coarse: it ends as soon as it is within AutoConfig.ARRIVE_TOLERANCE_DEG, with no retries.
        coarseTurn = !first;
        int retries = first ? (int) AutoConfig.TURN_RETRIES : 0;
        for (int attempt = 0; attempt <= retries; attempt++) {
            turnRetryAllowed = attempt < retries;
            turnStalled = false;
            navigate(TURN_POWER, x, y, target);
            if (aborted || !turnStalled) break;
        }
        turnRetryAllowed = false;
        coarseTurn = false;
    }

    private int turnCount;
    private boolean coarseTurn;
    private boolean turnRetryAllowed, turnStalled;

    /**
     * Camera-free absolute fix: drive gently into a wall, stop when the robot stalls against it, then reset
     * that coordinate and the heading to the known wall values (see WallBump). Example in runSequence():
     *   bump(WallBump.Wall.MINUS_X);   // press against the red wall, fixes x and heading
     * The robot must already be roughly square to that wall, with a clear straight path (not through the hive).
     */
    private void bump(WallBump.Wall wall) {
        if (!opModeIsActive() || aborted) return;
        double h = odometry.getHeadingDeg();
        double half = WallBump.faceHalf(wall, h, AutoConfig.ROBOT_HALF_LENGTH_IN, AutoConfig.ROBOT_HALF_WIDTH_IN, AutoConfig.BUMP_MAX_SKEW_DEG);
        if (Double.isNaN(half)) { abort("BUMP: NOT SQUARE TO WALL (heading " + Math.round(h) + ")"); return; }
        double aim = WallBump.aimCoordinate(wall, WALL_IN, half, AutoConfig.BUMP_OVERDRIVE_IN);
        boolean alongX = wall == WallBump.Wall.PLUS_X || wall == WallBump.Wall.MINUS_X;
        double tx = alongX ? aim : odometry.getX();
        double ty = alongX ? odometry.getY() : aim;
        goalX = tx;
        goalY = ty;
        navigate(AutoConfig.BUMP_POWER, tx, ty, h, true);
        if (aborted) return;
        WallBump.Result r = WallBump.compute(wall, odometry.getX(), odometry.getY(), odometry.getHeadingDeg(),
                WALL_IN, AutoConfig.ROBOT_HALF_LENGTH_IN, AutoConfig.ROBOT_HALF_WIDTH_IN, AutoConfig.BUMP_MAX_SKEW_DEG, AutoConfig.BUMP_REACH_TOL_IN);
        if (!r.ok) { abort("BUMP: " + r.reason); return; }
        odometry.setFieldPose(r.x, r.y, r.headingDeg);
        telemetry.addData("Wall bump", "pose fixed to x %.1f  y %.1f  h %.0f", r.x, r.y, r.headingDeg);
        telemetry.update();
    }

    // ===== Internals =====

    /** Drive straight to (x, y), holding the current heading. (The hive keep-out detours were removed.) */
    private void driveRouted(double power, double x, double y) {
        goalX = x;
        goalY = y;
        navigate(power, x, y, odometry.getHeadingDeg());
    }

    /** Push against any remaining motion (up to 0.3 s) so a move ends stopped instead of sliding. */
    private void brakeToStop() {
        odometry.resetBrake();
        double t0 = clock.seconds();
        while (opModeIsActive() && clock.seconds() - t0 < 0.3) {
            odometry.update();
            if (!odometry.activeBrake(voltageScale())) break;
        }
        drivetrain.stop();
    }

    boolean isAborted() { return aborted; }

    LimelightVision limelight() { return limelight; }

    /**
     * Switch the Limelight to `pipeline` and look for its target for up to `seconds`, with the robot stopped and the
     * odometry kept fresh. Switches back to the AprilTag pipeline afterwards. Returns the last sighting.
     */
    LimelightVision.Sighting lookFor(int pipeline, double seconds) {
        limelight.usePipeline(pipeline);
        LimelightVision.Sighting last = limelight.look(pipeline);
        double t0 = clock.seconds();
        while (opModeIsActive() && clock.seconds() - t0 < seconds) {
            odometry.update();
            drivetrain.stop();
            last = limelight.look(pipeline);
            if (last.seen) break;
            idle();
        }
        limelight.usePipeline((int) AutoConfig.LL_PIPELINE);
        return last;
    }

    /**
     * Absolute x/y from a flower sighting (FlowerFix), no AprilTags. The robot must be STOPPED (pose taken now must
     * match the frames). Picks which flower it is from the odometry estimate, solves the pose, and only blends it in
     * if it agrees with odometry within LL_FIX_MAX_JUMP_IN (one bad sighting can't teleport the robot).
     * With LL_FIX_APPLY = 0 (default until Flower Range Cal is done) it only reports what it WOULD do.
     */
    String flowerFix(LimelightVision.Sighting seen, double[][] flowers) {
        if (!seen.seen) return "no sighting";
        if (Double.isNaN(seen.spacingPx)) return "no post pair -> no range";
        double px = odometry.getX(), py = odometry.getY(), h = odometry.getHeadingDeg();
        int i = FlowerFix.pickFlower(flowers, px, py, h, seen.txDeg);
        if (i < 0) return "no flower";
        double facing = flowers[i][2] + 180.0;          // the robot faces the flower; the flower faces back at it
        FlowerFix.Result r = FlowerFix.solve(flowers[i][0], flowers[i][1], facing, h, seen.txDeg, seen.spacingPx,
                AutoConfig.LL_FLOWER_RANGE_K, AutoConfig.LL_CAMERA_FORWARD_IN, AutoConfig.LL_CAMERA_LEFT_IN,
                AutoConfig.LL_FIX_MAX_VIEW_DEG);
        if (!r.ok) return "flower " + (i + 1) + ": " + r.reason;
        if (r.rangeIn > AutoConfig.LL_FIX_MAX_RANGE_IN) {
            return String.format("flower %d: too far (%.0f in) for a good range", i + 1, r.rangeIn);
        }
        double dx = r.x - px, dy = r.y - py, jump = Math.hypot(dx, dy);
        String what = String.format("flower %d  range %.1f  view %.0f  fix (%.1f, %.1f)  odo off by %.1f in",
                i + 1, r.rangeIn, r.viewAngleDeg, r.x, r.y, jump);
        if (jump > AutoConfig.LL_FIX_MAX_JUMP_IN) return what + "  REJECTED (too far from odometry)";
        if (AutoConfig.LL_FIX_APPLY == 0) return what + "  (not applied: LL_FIX_APPLY = 0)";
        double g = Range.clip(AutoConfig.LL_FIX_GAIN, 0.0, 1.0);
        odometry.setFieldPose(px + g * dx, py + g * dy, h);   // heading stays: the Pinpoint IMU is better at it
        pauseUpdating(0.15);                                  // pose jumped: let velocity / readiness rebuild
        return what + String.format("  APPLIED %.0f%%", g * 100);
    }

    /**
     * Like lookFor, but never trusts one frame: collects up to LL_STEADY_FRAMES distinct frames that see the target
     * within `seconds` and returns their median tx / ty / ta. Fewer than LL_STEADY_MIN_FRAMES = not seen.
     * One jumpy frame (target box grabbing a floor blob) can no longer steer the robot.
     */
    LimelightVision.Sighting lookForSteady(int pipeline, double seconds) {
        limelight.usePipeline(pipeline);
        java.util.ArrayList<LimelightVision.Sighting> got = new java.util.ArrayList<>();
        LimelightVision.Sighting last = limelight.look(pipeline);
        double lastStamp = Double.NaN;
        double t0 = clock.seconds();
        int want = (int) AutoConfig.LL_STEADY_FRAMES;
        while (opModeIsActive() && clock.seconds() - t0 < seconds && got.size() < want) {
            odometry.update();
            drivetrain.stop();
            last = limelight.look(pipeline);
            if (last.seen && last.frameStamp != lastStamp) {   // only count NEW frames
                got.add(last);
                lastStamp = last.frameStamp;
            }
            idle();
        }
        limelight.usePipeline((int) AutoConfig.LL_PIPELINE);
        return LimelightVision.summarize(got, last);
    }

    /** > 0: use this arrival tolerance (inches) instead of AutoConfig.ARRIVE_TOLERANCE_IN for the next moves. */
    double arriveTolOverride = -1;

    /** Drive to (x, y) like drive(), but only accept it within `tolIn` (for stop points that must be exact). */
    void drivePrecise(double power, double x, double y, double tolIn) {
        arriveTolOverride = tolIn;
        try { drive(power, x, y); } finally { arriveTolOverride = -1; }
    }

    /** Wait `seconds` with the robot stopped while keeping the odometry fresh (a plain sleep makes it stale). */
    void pauseUpdating(double seconds) {
        double t0 = clock.seconds();
        while (opModeIsActive() && clock.seconds() - t0 < seconds) {
            odometry.update();
            drivetrain.stop();
            idle();
        }
    }

    private void waitUntilReady(double maxSeconds) {
        double t0 = clock.seconds();
        do {
            odometry.update();
            if (odometry.isReady()) return;
            idle();
        } while (opModeIsActive() && clock.seconds() - t0 < maxSeconds);
    }

    private boolean aborted;
    private String abortReason = "";

    /** Stop the whole routine: later drive()/turn() calls do nothing, so a bad estimate cannot compound. */
    private void abort(String reason) {
        if (aborted) return;
        aborted = true;
        if (light != null) light.set(StatusLight.State.ABORTED);
        abortReason = reason;
        odometry.cancelNavigation();
        drivetrain.stop();
    }

    private void navigate(double power, double x, double y, double headingDeg) {
        navigate(power, x, y, headingDeg, false);
    }

    /** contact = true: the target is deliberately past the contact point, so the robot is EXPECTED to stall on a wall. */
    private void navigate(double power, double x, double y, double headingDeg, boolean contact) {
        double limit = contact ? WALL_IN : FIELD_LIMIT_IN;
        if (Math.abs(x) > limit || Math.abs(y) > limit) {
            abort(String.format("TARGET OFF FIELD (%.0f, %.0f)", x, y));
            return;
        }
        odometry.setSpeedScale(Math.abs(power));
        odometry.setTurnFriction(AutoConfig.TURN_STATIC_POWER, AutoConfig.TURN_STATIC_TAPER_DEG);
        odometry.setSettleScale(AutoConfig.SETTLE_SCALE);
        double arriveTol = arriveTolOverride > 0 ? arriveTolOverride : AutoConfig.ARRIVE_TOLERANCE_IN;
        // The pose goes stale (odometry "not ready") if nothing polled the Pinpoint for > 0.25 s, e.g. after a
        // pause. Poll until it is fresh again before asking for the next move, instead of having it rejected.
        waitUntilReady(0.5);
        odometry.setNearFinishWindow(AutoConfig.FINISH_PAUSE_S, arriveTol,
                AutoConfig.ARRIVE_TOLERANCE_DEG);
        odometry.syncTargetsToPose();
        if (!odometry.setTarget(x, y, headingDeg)) {
            abort("MOVE REJECTED: " + odometry.getLastResult());
            return;
        }
        odometry.startNavigation();
        double start = clock.seconds();
        boolean timedOut = false;
        while (opModeIsActive() && odometry.isNavigating()) {
            odometry.update();
            if (odometry.updateNavigation(voltageScale())) drivetrain.stop();
            if (clock.seconds() - start > AutoConfig.MOVE_TIMEOUT_S) {
                odometry.cancelNavigation();
                timedOut = true;
                break;
            }
            // Wall guard: the ESTIMATE says we are past the field edge, so something is wrong.
            if (Math.abs(odometry.getX()) > WALL_IN || Math.abs(odometry.getY()) > WALL_IN) {
                abort(String.format("ESTIMATE OFF FIELD (%.0f, %.0f)", odometry.getX(), odometry.getY()));
                break;
            }
            if (coarseTurn && Math.abs(((headingDeg - odometry.getHeadingDeg()) % 360 + 540) % 360 - 180)
                    <= AutoConfig.ARRIVE_TOLERANCE_DEG) {
                odometry.cancelNavigation();             // close enough: skip the fine creep to 0.5 deg
                brakeToStop();
                return;
            }
            PoseDisplay.setExtras(batteryVolts(), gamepad1);
            LimelightVision.Reading ll = limelight.update(odometry.getHeadingDeg());
            PoseDisplay.setVision(ll);
            PoseDisplay.setTargetDistance(limelight.getTargetDistanceIn());
            light.set(StatusLight.State.MOVING);
            if (capture != null) capture.record(odometry.getX(), odometry.getY(), odometry.getHeadingDeg(), odometry.getTargetX(),
                    odometry.getTargetY(), goalX, goalY, batteryVolts());
            PoseDisplay.show(telemetry, odometry.getX(), odometry.getY(), odometry.getHeadingDeg(),
                    odometry.getTargetX(), odometry.getTargetY(), goalX, goalY);
            telemetry.addData("Limelight pose", limelightText(ll));
            telemetry.update();
        }
        brakeToStop();
        // Anything but ARRIVED (no progress, timeout, odometry fault, cancel) means the robot is not
        // where the routine assumes, so stop instead of running the next step from a wrong pose.
        String result = timedOut ? "MOVE TIMEOUT" : odometry.getLastResult();
        boolean stalled = result.startsWith("NO POSITION PROGRESS") || result.startsWith("DRIVE TIMEOUT")
                || result.startsWith("DRIVE HARD CAP");
        if (aborted || "ARRIVED".equals(result) || (contact && stalled)) return;
        // The controller's own ARRIVED needs ~0.24 in and 0.5 deg (TeleOp precision). A robot that stops a
        // fraction of an inch short trips its no-progress watchdog instead, which used to abort the whole loop.
        // Accept the move if it ended within AutoConfig.ARRIVE_TOLERANCE_IN / _DEG of the target.
        double posErr = Math.hypot(x - odometry.getX(), y - odometry.getY());
        double headErr = Math.abs(((headingDeg - odometry.getHeadingDeg()) % 360 + 540) % 360 - 180);
        if (posErr <= arriveTol && headErr <= AutoConfig.ARRIVE_TOLERANCE_DEG) return;
        if (turnRetryAllowed && result.startsWith("NO TURN PROGRESS")) { turnStalled = true; return; }
        abort(String.format("MOVE ENDED: %s (off by %.2f in, %.1f deg)", result, posErr, headErr));
    }

    private static String limelightText(LimelightVision.Reading r) {
        if (r == null) return "none";
        if (!r.valid || !r.hasPose) return r.valid ? "no pose (tags seen, no 3D position)" : r.status;
        return String.format("x %.1f  y %.1f  h %.0f  (%d tags, %s, %s hive)", r.x, r.y, r.headingDeg, r.tagCount,
                r.poseSource, r.hiveIsRed ? "RED" : "BLUE");
    }

    private boolean waitForOdometry() {
        double start = clock.seconds();
        while (opModeIsActive() && !odometry.isReady()) {
            odometry.update();
            odometry.updateReset();
            drivetrain.stop();
            if (clock.seconds() - start > AutoConfig.CALIBRATION_TIMEOUT_S) return false;
        }
        odometry.syncTargetsToPose();
        return opModeIsActive();
    }

    private double goalX = Double.NaN, goalY = Double.NaN;   // final destination of the current drive()

    /** Battery voltage for the Dashboard graph, or NaN if unavailable. */
    private double batteryVolts() {
        try {
            return voltageSensor == null ? Double.NaN : voltageSensor.getVoltage();
        } catch (RuntimeException ex) {
            return Double.NaN;
        }
    }

    private double voltageScale() {
        try {
            double v = voltageSensor == null ? Double.NaN : voltageSensor.getVoltage();
            if (Double.isNaN(v) || v < 8.0 || v > 16.8) return 1.0;
            return Range.clip(AutoConfig.NOMINAL_VOLTAGE / v, AutoConfig.MIN_VOLTAGE_SCALE, AutoConfig.MAX_VOLTAGE_SCALE);
        } catch (RuntimeException ex) {
            return 1.0;
        }
    }
}
