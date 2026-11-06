package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


/**
* Nonblocking Pinpoint navigation; inches and degrees.
* Field frame: origin at field center (see FieldStart), heading CCW-positive.
* Pinpoint is set to FieldStart after every reset, so X/Y/heading are field coordinates.
* D-pad target increments are along the FIELD axes (up/down = Y, right/left = X), never rotated by heading.
* Idle target follows the pose; a driver-set target is HELD until reached, finished, cancelled or synced.
* Calls to update() are owned by the OpMode; other methods never poll Pinpoint.
* Drivetrain.drive(): +forward, +strafe RIGHT, +turn CW (verify on your robot).
* Motor power normalization and BRAKE zero-power behavior belong in Drivetrain.
*/
public class Odometry2 {
   // UNVERIFIED hardware settings retained from the supplied code.
   // These offsets also occur in the manufacturer example; measure your own robot.
   // X offset: forward pod sideways position (+left). Y offset: lateral pod
   // forward position (+forward). Both are in mm relative to the tracking point.
   private static final double POD_X_OFFSET_MM = -84.0;
   private static final double POD_Y_OFFSET_MM = -168.0;
   private static final double STRAFE_DIRECTION = -1.0;
   private static final double TURN_DIRECTION = -1.0;


   // ---- NORMAL NAVIGATION (aggressive): Y-button navigation. Starting values, tune on the robot.
   // Old values are in the comments; restore them to dial back.
   private static final double TURN_KP = 0.020;        // was 0.012
   private static final double TURN_KD = 0.0020;       // was 0.001; derivative on measured angular velocity
   private static final double HOLD_KP = 0.020;        // was 0.012: heading hold while driving
   private static final double HOLD_KD = 0.0020;       // was 0.001
   private static final double TURN_STATIC = 0.025;
   // Turn friction feedforward actually used (default = TURN_STATIC tapered over 8 deg, exactly as before).
   // MainAuto2 raises it with setTurnFriction(); TeleOp never calls the setter, so it is unchanged.
   private double turnStatic = TURN_STATIC;
   private double turnStaticTaperDeg = 8.0;
   // Multiplies the arrival / turn settle waits (1.0 = TeleOp default). MainAuto2 uses 0.5 to decide twice as fast.
   private double settleScale = 1.0;
   // Near-target pause: once within nearDistIn / nearHeadDeg of the target, a stalled move is given up after
   // nearWindowS instead of the normal 1.25 s / 1.0 s. Off (-1) by default, so TeleOp is unchanged.
   private double nearWindowS = -1, nearDistIn = 0, nearHeadDeg = 0;
   private static final double TURN_SLEW_PER_S = 3.5;  // was 1.0
   private static final double TURN_MAX_POWER = 0.55;  // was 0.30
   private static final double HOLD_MAX_POWER = 0.45;  // was 0.25
   // Acceptance requirement, NOT a dead zone subtracted from proportional error.
   // 0.5 deg is a starting requirement; verify sensor noise and achievable accuracy.
   private static final double TURN_TOLERANCE_DEG = 0.5;
   private static final double DIST_KP = 0.040;         // power per inch; was 0.0203 (0.8 per metre)
   private static final double DIST_KD = 0.0085;        // damping, power per in/s; was 0.0038 (0.15 per m/s)
   private static final double DIST_STATIC = 0.10;      // was 0.08: helps the last inch break away
   private static final double STATIC_RAMP_IN = 0.025 / 0.0254;
   private static final double DIST_MAX_POWER = 0.9;   // was 0.6
   private static final double DIST_TOLERANCE_IN = 0.006 / 0.0254;
   private static final double POWER_SLEW_PER_S = 3.5;  // was 1.2
   private static final double STILL_SPEED_IPS = 0.035 / 0.0254;
   private static final double STILL_TURN_DPS = 4.0;
   private static final double ARRIVAL_SETTLE_S = 0.12;  // was 0.25
   private static final double TURN_SETTLE_S = 0.08;     // was 0.15
   // Settle timeout: was 3.0 s from the start of the turn; now 8.0 s counted ONLY while the heading
   // error is inside TURN_TOLERANCE_DEG. A robot turning the wrong way never accrues it, so the two
   // limits below are what stop it.
   // Turn direction lock: a target ~180 deg away has a coin-flip shortest direction. Near-180
   // turns pick CCW once at the start and keep it until the error is below the release angle.
   private static final double TURN_LOCK_ENTER_DEG = 175.0;
   private static final double TURN_LOCK_RELEASE_DEG = 20.0;
   // Braking envelope for turns: allowed speed = sqrt(2 * DECEL * |error|); extra brake above it.
   // STARTING VALUES - tune on the robot (lower DECEL / raise GAIN to brake earlier / harder).
   private static final double TURN_DECEL_DPS2 = 550.0;  // deg/s^2; was 300 (scaled with the higher power)
   private static final double TURN_BRAKE_GAIN = 0.0045; // power per deg/s over the envelope; was 0.002
   // Home-hold mode (X): return to FieldStart.HOME_* when moved away.
   // ---- HOME RETURN: used ONLY while home-hold is returning. These now equal the normal (aggressive)
   // values above by reference; replace a reference with a number to tune the return separately.
   private static final double HOME_HOLD_POS_DEADBAND_IN = 0.5;       // was 0.75
   private static final double HOME_HOLD_HEADING_DEADBAND_DEG = 1.5;  // was 2.0
   private static final double HOME_HOLD_DELAY_S = 0.1;      // was 0.3: must stay displaced this long
   private static final double HOME_HOLD_COOLDOWN_S = 0.5;   // was 1.0: pause after any return attempt
   private static final int HOME_HOLD_MAX_FAILS = 3;         // consecutive failed returns before lockout
   private static final double RET_DIST_MAX_POWER = DIST_MAX_POWER;     // was 0.6 x 0.5 scale = 0.30
   private static final double RET_DIST_KP = DIST_KP;          // power per inch (normal 0.0203)
   private static final double RET_DIST_KD = DIST_KD;         // power per in/s (normal 0.0038): more braking
   private static final double RET_POWER_SLEW_PER_S = POWER_SLEW_PER_S;   // normal 1.2: reach full power quicker
   private static final double RET_TURN_MAX_POWER = TURN_MAX_POWER;    // was 0.30 x 0.5 scale = 0.15
   private static final double RET_TURN_KP = TURN_KP;          // normal 0.012
   private static final double RET_TURN_KD = TURN_KD;         // normal 0.001
   private static final double RET_HOLD_KP = HOLD_KP;          // heading hold while driving (normal 0.012)
   private static final double RET_HOLD_KD = HOLD_KD;         // (normal 0.001)
   private static final double RET_HOLD_MAX_POWER = HOLD_MAX_POWER;    // normal 0.25
   private static final double RET_TURN_SLEW_PER_S = TURN_SLEW_PER_S;    // normal 1.0
   private static final double RET_TURN_DECEL_DPS2 = TURN_DECEL_DPS2;  // normal 300: envelope scaled with power
   private static final double RET_TURN_BRAKE_GAIN = TURN_BRAKE_GAIN;  // normal 0.002: brake harder over envelope
   private static final double RET_ARRIVAL_SETTLE_S = ARRIVAL_SETTLE_S;  // normal 0.25
   private static final double RET_TURN_SETTLE_S = TURN_SETTLE_S;     // normal 0.15
   private static final double TURN_TIMEOUT_S = 8.0;
   private static final double TURN_HARD_CAP_S = 10.0;  // absolute limit on a whole turn phase
   private static final double TURN_PROGRESS_EPS_DEG = 1.0;   // required error decrease ...
   private static final double TURN_PROGRESS_TIMEOUT_S = 1.0; // ... within this window, outside tolerance
   private static final double DRIVE_TIMEOUT_S = 10.0; // was 5.0; counts ONLY while inside DIST_TOLERANCE_IN
   private static final double DRIVE_HARD_CAP_S = 30.0; // absolute safety stop for a whole drive phase
   private static final double POSITION_PROGRESS_EPS_IN = 0.0015 / 0.0254;
   private static final double POSITION_PROGRESS_TIMEOUT_S = 1.25;
   private static final double RESET_STILL_S = 0.35;
   private static final double RESET_STOP_TIMEOUT_S = 3.0;
   private static final double RESET_READY_TIMEOUT_S = 5.0;
   private static final double RESET_MIN_WAIT_S = 0.50;
   private static final double VELOCITY_WINDOW_S = 0.04;
   private static final double MAX_SAMPLE_GAP_S = 0.25;
   private static final double STEP_IN = 1.0;
   private static final double HEADING_STEP_DEG = 5.0;


   private enum NavState { IDLE, TURNING, DRIVING }
   private enum ResetState { NONE, STOPPING, WAIT_READY }
   private final GoBildaPinpointDriver pinpoint;
   private Drivetrain2 drivetrain;
   private final ElapsedTime clock = new ElapsedTime();
   private Pose2D pose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);
   private String status = "NOT READ";
   private String lastResult = "NOT INITIALIZED";
   private NavState navState = NavState.IDLE;
   private ResetState resetState = ResetState.NONE;
   private boolean sampleValid;
   private boolean velocityValid;
   private boolean referenceReady;
   private double lastSampleTime = -1.0;
   private double velocityTime = -1.0;
   private double anchorTime = -1.0;
   private double anchorX, anchorY, anchorHeading;
   private double velocityX, velocityY, angularVelocity;
   private double targetX, targetY, targetHeading;
   private double phaseStarted, resetStarted;
   private int turnDirLock;           // 0 = none, +1 = locked CCW (near-180 turn)
   private double turnBrake;
   private boolean homeHold, homeReturn;   // home-hold mode on / currently returning
   private double speedScale = 1.0;    // caps drive/turn/hold power (autonomous test speed limit)
   private boolean manualDriving;          // driver sticks active: automatic return is paused
   private boolean homeLockedOut;          // too many failed returns; cleared by sticks or toggle
   private int homeFailCount;
   private double homeAwaySince = -1.0, homeCooldownUntil;
   private boolean targetHeld;      // driver set a target: do not follow the pose
   private double turnToleranceTime;   // seconds inside TURN_TOLERANCE_DEG during the turn phase
   private double bestTurnError = Double.POSITIVE_INFINITY, lastTurnProgressTime = -1.0;
   private double toleranceTime;    // seconds spent inside DIST_TOLERANCE_IN during the drive phase
   private double settledSince = -1.0;
   private double resetStillSince = -1.0;
   private double bestDistance = Double.POSITIVE_INFINITY;
   private double lastPositionProgressTime = -1.0;
   private double lastControlTime;
   private double commandX, commandY;
   private double turnCommand;
   private double turnEffort; // slew only the P + friction effort, never the D brake
   private double controlDt, turnP, turnD, turnFeedforward, turnRaw;
   private boolean turnClipped;


   public Odometry2(HardwareMap hardwareMap) { this(hardwareMap, null); }


   public Odometry2(HardwareMap hardwareMap, Drivetrain2 drivetrain) {
       pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
       this.drivetrain = drivetrain;
   }


   public void setDrivetrain(Drivetrain2 drivetrain) {
       cancelNavigation();
       this.drivetrain = drivetrain;
   }


   /** The robot must already be stationary during INIT. */
   public void init() {
       stopDrive();
       pinpoint.setOffsets(POD_X_OFFSET_MM, POD_Y_OFFSET_MM, DistanceUnit.MM);
       pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
       pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
               GoBildaPinpointDriver.EncoderDirection.REVERSED); // Y pod was counting down when moving left
       issueReset();
   }


   /** Request a reset; actual recalibration waits for a stationary robot. */
   public void reset() {
       if (isResetting()) return;
       cancelNavigation();
       stopDrive();
       resetState = ResetState.STOPPING;
       resetStarted = clock.seconds();
       resetStillSince = -1.0;
       lastResult = "RESET: WAITING FOR STOP";
   }


   public void cancelPendingReset() {
       if (resetState == ResetState.STOPPING) {
           resetState = ResetState.NONE;
           lastResult = "RESET CANCELLED BY DRIVER";
           resetStillSince = -1.0;
       }
   }


   // Start pose used when a reset completes. Defaults to FieldStart; setFieldPose() overrides it.
   private double startPoseX = FieldStart.X_IN;
   private double startPoseY = FieldStart.Y_IN;
   private double startPoseHeading = FieldStart.HEADING_DEG;


   /**
    * Declare the robot's TRUE field pose (inches, degrees) right now. Use it to set the start pose
    * at INIT when the robot is not at FieldStart, or to correct drift/relocalize from an absolute
    * reference (AprilTag, wall distance) mid-run. Safe to call repeatedly.
    */
   public void setFieldPose(double xIn, double yIn, double headingDeg) {
       if (!finite(xIn) || !finite(yIn) || !finite(headingDeg)) return;
       startPoseX = xIn;
       startPoseY = yIn;
       startPoseHeading = headingDeg;
       if (isResetting() || !referenceReady) return;   // applied when the reset completes
       applyFieldStartPose();
       copyPoseToTargets();
   }


   /** Pinpoint zeroed itself on reset; move it to the field-relative start pose. No extra I2C poll. */
   private void applyFieldStartPose() {
       Pose2D start = new Pose2D(DistanceUnit.INCH, startPoseX, startPoseY,
               AngleUnit.DEGREES, startPoseHeading);
       try {
           pinpoint.setPosition(start);
           pose = start;
           clearVelocity(); // pose jumped; do not derive velocity across the jump
       } catch (RuntimeException ex) {
           lastResult = "START POSE FAILED: " + ex.getClass().getSimpleName();
       }
   }


   private void issueReset() {
       stopDrive();
       referenceReady = false;
       sampleValid = false;
       clearVelocity();
       resetState = ResetState.WAIT_READY;
       resetStarted = clock.seconds();
       resetStillSince = -1.0;
       lastResult = "RESET: CALIBRATING -- KEEP STILL";
       try {
           pinpoint.resetPosAndIMU();
       } catch (RuntimeException ex) {
           resetState = ResetState.NONE;
           lastResult = "RESET COMMAND FAILED: " + ex.getClass().getSimpleName();
       }
   }


   /** Call after update() in init_loop and during an active reset. Never sleeps. */
   public void updateReset() {
       if (!isResetting()) return;
       stopDrive();
       double now = clock.seconds();
       if (resetState == ResetState.STOPPING) {
           if (now - resetStarted > RESET_STOP_TIMEOUT_S) {
               resetState = ResetState.NONE;
               lastResult = "RESET ABORTED: STOP NOT CONFIRMED";
               return;
           }
           if (isStationary()) {
               if (resetStillSince < 0) resetStillSince = now;
               if (now - resetStillSince >= RESET_STILL_S) issueReset();
           } else resetStillSince = -1.0;
           return;
       }
       if (now - resetStarted > RESET_READY_TIMEOUT_S) {
           resetState = ResetState.NONE;
           lastResult = "RESET FAILED: NO STABLE READY POSE";
           return;
       }
       // Do not accept a single ready-looking sample immediately after the command.
       if (now - resetStarted >= RESET_MIN_WAIT_S && isStationary()) {
           if (resetStillSince < 0) resetStillSince = now;
           if (now - resetStillSince >= RESET_STILL_S) {
               referenceReady = true;
               resetState = ResetState.NONE;
               applyFieldStartPose();
               copyPoseToTargets();
               lastResult = "RESET COMPLETE";
           }
       } else resetStillSince = -1.0;
   }


   /** Exactly one Pinpoint poll. Estimate velocity from cached poses, no extra I2C reads. */
   public void update() {
       double now = clock.seconds();
       double previousSampleTime = lastSampleTime;
       try {
           pinpoint.update();
           Pose2D next = pinpoint.getPosition();
           status = String.valueOf(pinpoint.getDeviceStatus());
           sampleValid = next != null && finite(next.getX(DistanceUnit.INCH))
                   && finite(next.getY(DistanceUnit.INCH))
                   && finite(next.getHeading(AngleUnit.DEGREES));
           if (sampleValid) pose = next;
       } catch (RuntimeException ex) {
           sampleValid = false;
           status = "READ FAILED: " + ex.getClass().getSimpleName();
       }
       lastSampleTime = clock.seconds();
       // Include time spent in the device read when checking a stalled control loop.
       boolean longGap = lastSampleTime - now > MAX_SAMPLE_GAP_S
               || (previousSampleTime >= 0.0
               && lastSampleTime - previousSampleTime > MAX_SAMPLE_GAP_S);
       if (!poseUsable() || longGap) {
           clearVelocity();
           resetStillSince = -1.0;
           if (isNavigating()) finish(longGap ? "CONTROL LOOP GAP" : "ODOMETRY FAULT");
           return;
       }
       now = lastSampleTime;
       if (anchorTime < 0) {
           setVelocityAnchor(now);
       } else if (now - anchorTime >= VELOCITY_WINDOW_S) {
           double dt = now - anchorTime;
           velocityX = (getX() - anchorX) / dt;
           velocityY = (getY() - anchorY) / dt;
           angularVelocity = wrapSignedDeg(getHeadingDeg() - anchorHeading) / dt;
           velocityValid = finite(velocityX) && finite(velocityY) && finite(angularVelocity);
           velocityTime = now;
           setVelocityAnchor(now);
       }
       updateTargetFollow();
       updateHomeHold();
   }

   /** Home-hold mode: if displaced from the home pose (and idle/ready), drive back and re-hold. */
   private void updateHomeHold() {
       if (!homeHold) return;
       double now = clock.seconds();
       if (manualDriving || homeLockedOut) { homeAwaySince = -1.0; return; }
       if (navState != NavState.IDLE || isResetting() || !isReady()) { homeAwaySince = -1.0; return; }
       double dist = Math.hypot(FieldStart.HOME_X_IN - getX(), FieldStart.HOME_Y_IN - getY());
       double headingErr = Math.abs(wrapSignedDeg(FieldStart.HOME_HEADING_DEG - getHeadingDeg()));
       boolean away = dist > HOME_HOLD_POS_DEADBAND_IN || headingErr > HOME_HOLD_HEADING_DEADBAND_DEG;
       if (!away) { homeAwaySince = -1.0; return; }
       if (homeAwaySince < 0.0) homeAwaySince = now;
       if (now - homeAwaySince < HOME_HOLD_DELAY_S || now < homeCooldownUntil) return;
       setHomeTarget();
       homeReturn = true;
       startNavigation();
       if (!isNavigating()) { homeReturn = false; recordHomeFailure(); }
       else lastResult = "HOME HOLD: RETURNING";
   }

   private void setHomeTarget() {
       targetX = FieldStart.HOME_X_IN;
       targetY = FieldStart.HOME_Y_IN;
       targetHeading = normalize360(FieldStart.HOME_HEADING_DEG);
       targetHeld = true;
   }

   /** X button: toggle home-hold. Turning it off also cancels a return in progress. */
   public void toggleHomeHold() {
       if (homeHold) {
           homeHold = false;
           homeAwaySince = -1.0;
           homeLockedOut = false;
           homeFailCount = 0;
           cancelNavigation();
           lastResult = "HOME HOLD OFF";
           return;
       }
       if (isResetting() || !isReady()) { lastResult = "NOT READY FOR HOME HOLD"; return; }
       if (isNavigating()) return;
       homeHold = true;
       homeAwaySince = -1.0;
       homeLockedOut = false;
       homeFailCount = 0;
       homeCooldownUntil = 0.0;
       setHomeTarget();
       lastResult = "HOME HOLD ON";
   }

   /**
    * Call every loop with whether the driver sticks are active. While true the automatic return is
    * paused (the driver always wins); the mode stays ON and resumes after the sticks are released.
    * Driver input also clears a failed-return lockout.
    */
   public void setManualDriving(boolean active) {
       manualDriving = active;
       if (active) { homeLockedOut = false; homeFailCount = 0; }
   }

   private void recordHomeFailure() {
       if (++homeFailCount >= HOME_HOLD_MAX_FAILS) {
           homeLockedOut = true;
           lastResult = "HOME HOLD: RETURN FAILED -- RETRYING PAUSED";
       }
   }

   public boolean isHomeHold() { return homeHold; }

   public String getHomeHoldState() {
       if (!homeHold) return "OFF";
       if (homeLockedOut) return "ON (LOCKED OUT: return failed, move sticks or toggle X)";
       if (manualDriving) return "ON (paused: manual)";
       if (homeReturn) return "ON (returning)";
       return "ON";
   }

   /** Heading error for control. A near-180 turn keeps its starting direction (no mid-turn flip). */
   private double lockedHeadingError() {
       double e = wrapSignedDeg(targetHeading - getHeadingDeg());
       if (turnDirLock == 0) return e;
       if (Math.abs(e) < TURN_LOCK_RELEASE_DEG) { turnDirLock = 0; return e; }
       if (turnDirLock > 0 && e < 0.0) e += 360.0;
       else if (turnDirLock < 0 && e > 0.0) e -= 360.0;
       return e;
   }

   /** Idle: while stationary, the target follows the pose unless the driver set one (held). */
   private void updateTargetFollow() {
       if (homeHold) return;       // home-hold keeps the target at home
       if (navState != NavState.IDLE || isResetting() || !referenceReady || !motionUsable()) return;
       if (!isStationary()) return;
       if (targetHeld) {
           boolean reached = Math.hypot(targetX - getX(), targetY - getY()) <= DIST_TOLERANCE_IN
                   && Math.abs(wrapSignedDeg(targetHeading - getHeadingDeg())) <= TURN_TOLERANCE_DEG;
           if (!reached) return;   // keep the driver's target
       }
       copyPoseToTargets();        // also releases the hold
   }


   private void setVelocityAnchor(double now) {
       anchorTime = now;
       anchorX = getX();
       anchorY = getY();
       anchorHeading = getHeadingDeg();
   }


   private void clearVelocity() {
       velocityValid = false;
       anchorTime = -1.0;
       velocityTime = -1.0;
       velocityX = velocityY = angularVelocity = 0.0;
   }


   private boolean poseUsable() {
       return sampleValid && "READY".equals(status) && lastSampleTime >= 0.0
               && clock.seconds() - lastSampleTime <= MAX_SAMPLE_GAP_S;
   }


   private boolean motionUsable() {
       return poseUsable() && velocityValid && velocityTime >= 0.0
               && clock.seconds() - velocityTime <= MAX_SAMPLE_GAP_S;
   }


   private boolean isStationary() {
       return motionUsable() && Math.hypot(velocityX, velocityY) <= STILL_SPEED_IPS
               && Math.abs(angularVelocity) <= STILL_TURN_DPS;
   }


   public boolean isReady() { return referenceReady && !isResetting() && motionUsable(); }
   public boolean isResetting() { return resetState != ResetState.NONE; }
   public boolean canDriveManually() { return !isResetting() && !"CALIBRATING".equals(status); }


   public void processInputs(boolean up, boolean down, boolean left, boolean right,
                             boolean x, boolean b, boolean y) {
       if (isNavigating() || isResetting()) return;
       if (homeHold) {
           if (up || down || left || right || x || b || y) lastResult = "HOME HOLD ON: targets locked";
           return;
       }
       if (!isReady()) {
           if (up || down || left || right || x || b || y) lastResult = "NOT READY FOR NAVIGATION";
           return;
       }
       // Field axes, not rotated by heading: a Y step can never change X (and vice versa).
       double dx = ((right ? 1 : 0) - (left ? 1 : 0)) * STEP_IN;
       double dy = ((up ? 1 : 0) - (down ? 1 : 0)) * STEP_IN;
       double dh = ((x ? 1 : 0) - (b ? 1 : 0)) * HEADING_STEP_DEG;
       targetX += dx;
       targetY += dy;
       targetHeading = normalize360(targetHeading + dh);
       if (dx != 0.0 || dy != 0.0 || dh != 0.0) targetHeld = true;
       if (y) startNavigation();
   }


   public void startNavigation() {
       if (isNavigating()) return;
       if (drivetrain == null || !isReady() || !targetsFinite()) {
           lastResult = drivetrain == null ? "NO DRIVETRAIN" : "NOT READY / INVALID TARGET";
           stopDrive();
           return;
       }
       double distance = Math.hypot(targetX - getX(), targetY - getY());
       // A position-changing command must be allowed to translate and turn at
       // the same time. Pure heading changes still use the dedicated turn phase.
       navState = distance > DIST_TOLERANCE_IN ? NavState.DRIVING : NavState.TURNING;
       phaseStarted = lastControlTime = clock.seconds();
       settledSince = -1.0;
       commandX = commandY = 0.0;
       toleranceTime = 0.0;
       turnToleranceTime = 0.0;
       double e0 = wrapSignedDeg(targetHeading - getHeadingDeg());
       turnDirLock = Math.abs(e0) >= TURN_LOCK_ENTER_DEG ? 1 : 0; // near-180: always CCW
       bestTurnError = Math.abs(e0);
       lastTurnProgressTime = phaseStarted;
       bestDistance = distance;
       lastPositionProgressTime = phaseStarted;
       lastResult = "RUNNING";
       stopDrive();
   }


   /**
    * Friction feedforward for turns: staticPower at errors >= taperDeg, tapering to 0 at the target. The default
    * (0.025 over 8 deg) is too weak to keep a mecanum robot rotating in the last few degrees, so it stalls short.
    */
   private static final double BRAKE_KV = 0.012;   // power per in/s of motion, pushing against it
   private static final double BRAKE_KW = 0.003;   // power per deg/s of rotation, pushing against it
   private static final double BRAKE_MAX = 0.6;

   /**
    * Active brake: drive briefly AGAINST the measured motion so the robot stops instead of sliding (BRAKE zero-power
    * mode alone only shorts the motors, which is weak on a heavy mecanum robot). Call every loop while stopping.
    * Returns true while still braking, false (motors zeroed) once the robot is still or the pose is unusable.
    */
   public boolean activeBrake(double voltageScale) {
       if (drivetrain == null || !motionUsable() || isStationary()) {
           stopDrive();
           return false;
       }
       double speed = Math.hypot(velocityX, velocityY);
       double spin = Math.abs(angularVelocity);
       // Done: slow enough that BRAKE mode alone holds it. Pushing on tiny, noisy velocities is what made the
       // robot creep after the sticks were released.
       if (speed < BRAKE_MIN_SPEED_IPS && spin < BRAKE_MIN_SPIN_DPS) {
           stopDrive();
           return false;
       }
       if (brakeStartSpeed < 0) {
           brakeStartSpeed = speed;
           brakeStartSpin = spin;
       } else if (speed > brakeStartSpeed + BRAKE_RISE_IPS || spin > brakeStartSpin + BRAKE_RISE_DPS) {
           // Braking is making it go FASTER (wrong sign or bad velocity estimate): stop pushing immediately rather
           // than drive the robot with no stick input.
           stopDrive();
           brakeStartSpeed = Double.POSITIVE_INFINITY;   // stays off until resetBrake()
           return false;
       }
       if (brakeStartSpeed == Double.POSITIVE_INFINITY) { stopDrive(); return false; }
       double theta = Math.toRadians(getHeadingDeg());
       double vForward = velocityX * Math.cos(theta) + velocityY * Math.sin(theta);
       double vLeft = -velocityX * Math.sin(theta) + velocityY * Math.cos(theta);
       double forward = Range.clip(-BRAKE_KV * vForward, -BRAKE_MAX, BRAKE_MAX);
       double left = Range.clip(-BRAKE_KV * vLeft, -BRAKE_MAX, BRAKE_MAX);
       double turn = Range.clip(-BRAKE_KW * angularVelocity, -BRAKE_MAX, BRAKE_MAX);
       drivetrain.drive(forward, STRAFE_DIRECTION * left, TURN_DIRECTION * turn, voltageScale);
       return true;
   }

   /** Start a new brake event (call when the sticks are released / a move ends). */
   public void resetBrake() { brakeStartSpeed = -1; brakeStartSpin = -1; }

   private static final double BRAKE_MIN_SPEED_IPS = 3.0;
   private static final double BRAKE_MIN_SPIN_DPS = 10.0;
   private static final double BRAKE_RISE_IPS = 2.0;
   private static final double BRAKE_RISE_DPS = 10.0;
   private double brakeStartSpeed = -1, brakeStartSpin = -1;

   public void setNearFinishWindow(double seconds, double withinIn, double withinDeg) {
       nearWindowS = seconds;
       nearDistIn = withinIn;
       nearHeadDeg = withinDeg;
   }

   public void setSettleScale(double scale) { settleScale = Range.clip(scale, 0.1, 1.0); }

   public void setTurnFriction(double staticPower, double taperDeg) {
       turnStatic = Range.clip(staticPower, 0.0, 0.3);
       turnStaticTaperDeg = Math.max(0.5, taperDeg);
   }

   /** Scale all drive/turn/hold power caps (0.1 to 1.0). Used by autonomous tests for low speed. */
   public void setSpeedScale(double scale) {
       speedScale = Range.clip(scale, 0.1, 1.0);
   }

   /**
    * Set an absolute field target (inches, degrees) for a programmatic navigation. Returns false
    * (and sets lastResult) if not ready, resetting, navigating, or home-hold is on.
    */
   public boolean setTarget(double xIn, double yIn, double headingDeg) {
       if (!finite(xIn) || !finite(yIn) || !finite(headingDeg)) { lastResult = "INVALID TARGET"; return false; }
       if (isNavigating() || isResetting()) { lastResult = "BUSY"; return false; }
       if (homeHold) { lastResult = "HOME HOLD ON: targets locked"; return false; }
       if (!isReady()) { lastResult = "NOT READY FOR NAVIGATION"; return false; }
       targetX = xIn;
       targetY = yIn;
       targetHeading = normalize360(headingDeg);
       targetHeld = true;
       return true;
   }

   /** Set (and hold) the target to the field home pose. Press Y to navigate there. */
   public void goHome() {
       if (isNavigating() || isResetting()) return;
       if (homeHold) { lastResult = "HOME HOLD ON: targets locked"; return; }
       if (!isReady()) {
           lastResult = "NOT READY FOR NAVIGATION";
           return;
       }
       targetX = FieldStart.HOME_X_IN;
       targetY = FieldStart.HOME_Y_IN;
       targetHeading = normalize360(FieldStart.HOME_HEADING_DEG);
       targetHeld = true;
       lastResult = "TARGET SET TO HOME";
   }


   public void cancelNavigation() {
       if (isNavigating()) finish("CANCELLED");
   }


   public void syncTargetsToPose() {
       cancelNavigation();
       if (!isReady()) {
           lastResult = "CANNOT SYNC: ODOMETRY NOT READY";
           return;
       }
       copyPoseToTargets();
       lastResult = "TARGET SYNCED";
   }


   private void copyPoseToTargets() {
       if (homeHold) { setHomeTarget(); return; }   // home-hold overrides every other target
       targetX = getX();
       targetY = getY();
       targetHeading = normalize360(getHeadingDeg());
       targetHeld = false;
   }


   /** True means idle/finished; caller may command zero. Every active branch writes motors. */
   public boolean updateNavigation(double voltageScale) {
       if (navState == NavState.IDLE) return true;
       double now = clock.seconds();
       double dt = now - lastControlTime;
       lastControlTime = now;
       if (drivetrain == null || !isReady() || !targetsFinite()
               || !finite(voltageScale) || voltageScale <= 0.0
               || !finite(dt) || dt < 0.0 || dt > MAX_SAMPLE_GAP_S) {
           finish("NAVIGATION ABORTED: INVALID STATE / DATA");
           return true;
       }
       controlDt = dt;
       if (navState == NavState.TURNING) handleTurning(now, dt, voltageScale);
       else handleDriving(now, dt, voltageScale);
       return navState == NavState.IDLE;
   }


   private void handleTurning(double now, double dt, double voltageScale) {
       double error = lockedHeadingError();
       boolean inTolerance = Math.abs(error) <= TURN_TOLERANCE_DEG;
       // 1) Settle timeout: only runs while inside the heading tolerance.
       if (inTolerance) turnToleranceTime += dt;
       if (turnToleranceTime > TURN_TIMEOUT_S) {
           finish("TURN TIMEOUT"); // never translate after a failed turn
           return;
       }
       // 2) Absolute cap on the whole turn phase.
       if (now - phaseStarted > TURN_HARD_CAP_S) {
           finish("TURN HARD CAP");
           return;
       }
       // 3) No-progress watchdog: outside tolerance, the error must keep shrinking.
       if (inTolerance) {
           lastTurnProgressTime = now;
           bestTurnError = Math.abs(error);
       } else if (Math.abs(error) + TURN_PROGRESS_EPS_DEG < bestTurnError) {
           bestTurnError = Math.abs(error);
           lastTurnProgressTime = now;
       } else if (lastTurnProgressTime >= 0.0 && now - lastTurnProgressTime
               > (nearWindowS > 0 && Math.abs(error) <= nearHeadDeg ? nearWindowS : TURN_PROGRESS_TIMEOUT_S)) {
           finish("NO TURN PROGRESS -- CHECK TURN SIGN");
           return;
       }
       if (inTolerance && isStationary()) {
           stopDrive();
           if (settledSince < 0) settledSince = now;
           if (now - settledSince >= pick(TURN_SETTLE_S, RET_TURN_SETTLE_S) * settleScale) {
               navState = NavState.DRIVING;
               phaseStarted = now;
               settledSince = -1.0;
               commandX = commandY = 0.0;
               toleranceTime = 0.0;
               bestDistance = Math.hypot(targetX - getX(), targetY - getY());
               lastPositionProgressTime = now;
           }
           return;
       }
       settledSince = -1.0;
       drivetrain.drive(0, 0, TURN_DIRECTION * turnPower(error,
               pick(TURN_KP, RET_TURN_KP), pick(TURN_KD, RET_TURN_KD),
               pick(TURN_MAX_POWER, RET_TURN_MAX_POWER) * speedScale, dt, true), voltageScale);
   }


   // Normal vs aggressive (home return) selectors.
   private double pick(double normal, double homeValue) { return homeReturn ? homeValue : normal; }

   private double turnPower(double error, double kp, double kd, double limit, double dt, boolean brake) {
       // The full normalized error drives P. Tolerance is used ONLY for settling.
       turnP = kp * error;
       // Deg/s from wrapped pose differences: no target-change derivative kick.
       turnD = -kd * angularVelocity;
       // Braking envelope (turn phase only): if spinning toward the target faster than it can stop
       // in the remaining angle, add extra opposing power.
       turnBrake = 0.0;
       if (brake && angularVelocity * error > 0.0) {
           double allowed = Math.sqrt(2.0 * pick(TURN_DECEL_DPS2, RET_TURN_DECEL_DPS2) * Math.abs(error));
           double excess = Math.abs(angularVelocity) - allowed;
           if (excess > 0.0) turnBrake = -Math.signum(error) * pick(TURN_BRAKE_GAIN, RET_TURN_BRAKE_GAIN) * excess;
       }
       turnFeedforward = 0.0;
       double pd = turnP + turnD;
       if (Math.abs(error) > TURN_TOLERANCE_DEG && Math.abs(angularVelocity) < 8.0
               && pd * error > 0.0) {
           // Retained tapered friction model. This is NOT a guaranteed minimum
           // usable motor power: measure breakaway power before changing it.
           turnFeedforward = Math.copySign(
                   turnStatic * Math.min(Math.abs(error) / turnStaticTaperDeg, 1.0), error);
       }
       double desiredEffort = Range.clip(turnP + turnFeedforward, -limit, limit);
       // Remove obsolete effort immediately when the target correction reverses.
       if (desiredEffort * turnEffort < 0.0) turnEffort = 0.0;
       if (Math.abs(desiredEffort) <= Math.abs(turnEffort)) {
           turnEffort = desiredEffort; // do not delay reducing propulsion
       } else {
           double step = pick(TURN_SLEW_PER_S, RET_TURN_SLEW_PER_S) * Math.min(dt, 0.05);
           turnEffort += Range.clip(desiredEffort - turnEffort, -step, step);
       }
       // Apply velocity damping AFTER the propulsion slew limiter so braking
       // can take effect on this control update, even while propulsion ramps.
       turnRaw = turnEffort + turnD + turnBrake;
       turnCommand = Range.clip(turnRaw, -limit, limit);
       turnClipped = Math.abs(turnRaw) > limit;
       return turnCommand;
   }

   private void handleDriving(double now, double dt, double voltageScale) {
       double ex = targetX - getX(), ey = targetY - getY();
       double distance = Math.hypot(ex, ey);
       double headingError = lockedHeadingError();
       boolean positionReached = distance <= DIST_TOLERANCE_IN;
       // The settle timeout only runs while inside the allowed position error. Outside it, the
       // no-progress watchdog below stops a robot that is moving the wrong way, and a long
       // absolute cap stops anything else.
       if (positionReached) toleranceTime += dt;
       if (toleranceTime > DRIVE_TIMEOUT_S) {
           finish("DRIVE TIMEOUT");
           return;
       }
       if (now - phaseStarted > DRIVE_HARD_CAP_S) {
           finish("DRIVE HARD CAP");
           return;
       }
       if (positionReached && Math.abs(headingError) <= TURN_TOLERANCE_DEG && isStationary()) {
           stopDrive();
           commandX = commandY = 0.0;
           if (settledSince < 0) settledSince = now;
           if (now - settledSince >= pick(ARRIVAL_SETTLE_S, RET_ARRIVAL_SETTLE_S) * settleScale) finish("ARRIVED");
           return;
       }
       settledSince = -1.0;

       if (positionReached) {
           // Position is already good: stop translating and finish the requested
           // heading correction in place. Refresh the watchdog so a small amount
           // of pose noise outside the tolerance cannot cause an immediate abort.
           commandX = commandY = 0.0;
           lastPositionProgressTime = now;
       } else {
           // Track actual field-position progress while translation is active. If
           // a physical sign/configuration is wrong, distance typically stops
           // decreasing and this aborts instead of allowing a prolonged orbit.
           if (distance + POSITION_PROGRESS_EPS_IN < bestDistance) {
               bestDistance = distance;
               lastPositionProgressTime = now;
           } else if (lastPositionProgressTime >= 0.0
                   && now - lastPositionProgressTime
                   > (nearWindowS > 0 && distance <= nearDistIn ? nearWindowS : POSITION_PROGRESS_TIMEOUT_S)) {
               finish("NO POSITION PROGRESS -- CHECK POD / STRAFE SIGNS");
               return;
           }

           double speed = pick(DIST_KP, RET_DIST_KP) * distance
                   + DIST_STATIC * Math.min(distance / STATIC_RAMP_IN, 1.0);
           // Signed damping can oppose travel to brake; no hard minimum power floor.
           double kd = pick(DIST_KD, RET_DIST_KD);
           double requestedX = speed * ex / distance - kd * velocityX;
           double requestedY = speed * ey / distance - kd * velocityY;
           double magnitude = Math.hypot(requestedX, requestedY);
           double distCap = pick(DIST_MAX_POWER, RET_DIST_MAX_POWER) * speedScale;
           if (magnitude > distCap) {
               requestedX *= distCap / magnitude;
               requestedY *= distCap / magnitude;
           }
           double dx = requestedX - commandX, dy = requestedY - commandY;
           double delta = Math.hypot(dx, dy);
           double allowed = pick(POWER_SLEW_PER_S, RET_POWER_SLEW_PER_S) * Math.min(dt, 0.05);
           double ratio = delta > allowed && delta > 0.0 ? allowed / delta : 1.0;
           commandX += dx * ratio;
           commandY += dy * ratio;
       }
       double theta = Math.toRadians(getHeadingDeg());
       double forward = commandX * Math.cos(theta) + commandY * Math.sin(theta);
       double left = -commandX * Math.sin(theta) + commandY * Math.cos(theta);
       double turn = turnPower(headingError, pick(HOLD_KP, RET_HOLD_KP), pick(HOLD_KD, RET_HOLD_KD),
               pick(HOLD_MAX_POWER, RET_HOLD_MAX_POWER) * speedScale, dt, false);
       drivetrain.drive(forward, STRAFE_DIRECTION * left, TURN_DIRECTION * turn, voltageScale);
   }


   private void finish(String result) {
       navState = NavState.IDLE;
       lastResult = result;
       settledSince = -1.0;
       commandX = commandY = 0.0;
       bestDistance = Double.POSITIVE_INFINITY;
       lastPositionProgressTime = -1.0;
       toleranceTime = 0.0;
       turnToleranceTime = 0.0;
       bestTurnError = Double.POSITIVE_INFINITY;
       lastTurnProgressTime = -1.0;
       turnDirLock = 0;
       homeReturn = false;
       homeCooldownUntil = clock.seconds() + HOME_HOLD_COOLDOWN_S;
       if (homeHold) {
           setHomeTarget();                                      // always keep the target at home
           if ("ARRIVED".equals(result)) { homeFailCount = 0; homeLockedOut = false; }
           else if (!"CANCELLED".equals(result)) recordHomeFailure(); // driver cancels are not failures
       }
       targetHeld = homeHold;
       stopDrive();
   }


   private void stopDrive() {
       turnCommand = turnEffort = 0.0;
       turnP = turnD = turnFeedforward = turnRaw = turnBrake = 0.0;
       turnClipped = false;
       if (drivetrain != null) drivetrain.drive(0, 0, 0, 1.0);
   }


   private boolean targetsFinite() {
       return finite(targetX) && finite(targetY) && finite(targetHeading);
   }


   // Programmatic adjustments stay field-relative; reject changes during motion/reset.
   public void adjustTargetX(double delta) {
       if (!homeHold && !isNavigating() && isReady() && finite(delta) && finite(targetX + delta)) { targetX += delta; targetHeld = true; }
   }
   public void adjustTargetY(double delta) {
       if (!homeHold && !isNavigating() && isReady() && finite(delta) && finite(targetY + delta)) { targetY += delta; targetHeld = true; }
   }
   public void adjustTargetHeading(double delta) {
       if (!homeHold && !isNavigating() && isReady() && finite(delta) && finite(targetHeading + delta))
           { targetHeading = normalize360(targetHeading + delta); targetHeld = true; }
   }
   public double getX() { return pose.getX(DistanceUnit.INCH); }
   public double getY() { return pose.getY(DistanceUnit.INCH); }
   /** Absolute heading for drivers/targets: 0 <= heading < 360, CCW-positive. */
   public double getHeadingDeg() { return normalize360(pose.getHeading(AngleUnit.DEGREES)); }
   public double getTargetX() { return targetX; }
   public double getTargetY() { return targetY; }
   public double getTargetHeading() { return targetHeading; }
   public String getStatus() { return status; }
   public String getLastResult() { return lastResult; }
   public String getNavigationState() { return navState.name(); }
   public String getResetState() { return resetState.name(); }
   public double getErrorX() { return targetX - getX(); }
   public double getErrorY() { return targetY - getY(); }
   public double getDistanceError() { return Math.hypot(getErrorX(), getErrorY()); }
   public double getHeadingErrorDeg() { return wrapSignedDeg(targetHeading - getHeadingDeg()); }
   public double getVelocityX() { return velocityX; }
   public double getVelocityY() { return velocityY; }
   public double getAngularVelocityDegPerSec() { return angularVelocity; }
   public boolean isMotionEstimateUsable() { return motionUsable(); }
   public double getCommandX() { return commandX; }
   public double getCommandY() { return commandY; }
   public double getTurnP() { return turnP; }
   public double getTurnD() { return turnD; }
   public double getTurnFeedforward() { return turnFeedforward; }
   public double getTurnRaw() { return turnRaw; }
   public double getTurnCommand() { return turnCommand; }
   public double getControlDt() { return controlDt; }
   public boolean isTurnClipped() { return turnClipped; }
   /** Historical result of the last operation, not a live check of the current pose. */
   public boolean hasArrived() { return navState == NavState.IDLE && "ARRIVED".equals(lastResult); }
   public boolean isTargetHeld() { return targetHeld; }
   public boolean isNavigating() { return navState != NavState.IDLE; }


   public void addTelemetry(Telemetry telemetry) {
       telemetry.addData("Pinpoint", status);
       telemetry.addData("Navigation ready", isReady());
       telemetry.addData("Pose (in / deg, field)", "X %.3f Y %.3f H %.1f", getX(), getY(), getHeadingDeg());
       telemetry.addData("Target (in / deg, field)", "X %.3f Y %.3f H %.1f", targetX, targetY, targetHeading);
       telemetry.addData("HOME HOLD", getHomeHoldState());
       telemetry.addData("Target mode", targetHeld ? "HELD (driver set)" : "FOLLOWING pose");
       telemetry.addData("Error", "%.3f in / %.1f deg",
               Math.hypot(targetX - getX(), targetY - getY()),
               wrapSignedDeg(targetHeading - getHeadingDeg()));
       telemetry.addData("Estimated motion", "%.3f in/s / %.1f deg/s (%s)",
               Math.hypot(velocityX, velocityY), angularVelocity, motionUsable() ? "valid" : "unavailable");
       telemetry.addData("Navigation", "%s / %s", navState, lastResult);
       telemetry.addData("Reset", resetState);
       telemetry.addData("Turn P / D / FF", "%.4f / %.4f / %.4f", turnP, turnD, turnFeedforward);
       telemetry.addData("Turn effort / raw / CCW out", "%.4f / %.4f / %.4f",
               turnEffort, turnRaw, turnCommand);
       telemetry.addData("Turn clipped / control dt", "%s / %.4f s", turnClipped, controlDt);
       telemetry.addData("Velocity age", "%.3f s",
               velocityTime < 0 ? -1.0 : clock.seconds() - velocityTime);
       telemetry.addData("Heading acceptance", "%.2f deg", TURN_TOLERANCE_DEG);
       telemetry.addData("Hardware settings", "VERIFY offsets %.1f / %.1f mm; 4-bar; FWD/REV",
               POD_X_OFFSET_MM, POD_Y_OFFSET_MM);
   }


   private static boolean finite(double value) { return !Double.isNaN(value) && !Double.isInfinite(value); }

   /** Signed shortest angular delta in [-180, 180). Use this for control only. */
   private static double wrapSignedDeg(double degrees) {
       return AngleUnit.normalizeDegrees(degrees);
   }

   /** Absolute driver-facing angle in [0, 360). */
   private static double normalize360(double degrees) {
       double normalized = degrees % 360.0;
       if (normalized < 0.0) normalized += 360.0;
       return normalized == 0.0 || normalized >= 360.0 ? 0.0 : normalized;
   }
}
