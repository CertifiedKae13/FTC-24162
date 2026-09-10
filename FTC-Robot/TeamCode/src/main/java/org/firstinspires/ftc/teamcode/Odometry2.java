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
* Nonblocking Pinpoint navigation; metres and degrees.
* Field axes at reset: +X forward, +Y left, heading CCW-positive.
* D-pad target increments are robot-relative using the current measured heading.
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


   // Starting values, requiring tuning on the actual drivetrain.
   private static final double TURN_KP = 0.012;
   private static final double TURN_KD = 0.001; // derivative on measured angular velocity
   private static final double HOLD_KP = 0.012;
   private static final double HOLD_KD = 0.001;
   private static final double TURN_STATIC = 0.025;
   private static final double TURN_MAX_POWER = 0.30;
   private static final double HOLD_MAX_POWER = 0.25;
   // Acceptance requirement, NOT a dead zone subtracted from proportional error.
   // 0.5 deg is a starting requirement; verify sensor noise and achievable accuracy.
   private static final double TURN_TOLERANCE_DEG = 0.5;
   private static final double DIST_KP = 0.8;
   private static final double DIST_KD = 0.15; // field velocity damping
   private static final double DIST_STATIC = 0.08;
   private static final double STATIC_RAMP_M = 0.025;
   private static final double DIST_MAX_POWER = 0.6;
   private static final double DIST_TOLERANCE_M = 0.006;
   private static final double POWER_SLEW_PER_S = 1.2;
   private static final double STILL_SPEED_MPS = 0.035;
   private static final double STILL_TURN_DPS = 4.0;
   private static final double ARRIVAL_SETTLE_S = 0.25;
   private static final double TURN_SETTLE_S = 0.15;
   private static final double TURN_TIMEOUT_S = 3.0;
   private static final double DRIVE_TIMEOUT_S = 5.0;
   private static final double RESET_STILL_S = 0.35;
   private static final double RESET_STOP_TIMEOUT_S = 3.0;
   private static final double RESET_READY_TIMEOUT_S = 5.0;
   private static final double RESET_MIN_WAIT_S = 0.50;
   private static final double VELOCITY_WINDOW_S = 0.04;
   private static final double MAX_SAMPLE_GAP_S = 0.25;
   private static final double STEP_M = 0.0254;
   private static final double HEADING_STEP_DEG = 5.0;


   private enum NavState { IDLE, TURNING, DRIVING }
   private enum ResetState { NONE, STOPPING, WAIT_READY }
   private final GoBildaPinpointDriver pinpoint;
   private Drivetrain2 drivetrain;
   private final ElapsedTime clock = new ElapsedTime();
   private Pose2D pose = new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0);
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
   private double settledSince = -1.0;
   private double resetStillSince = -1.0;
   private double lastControlTime;
   private double commandX, commandY;
   private double turnCommand;
   private double turnEffort; // slew only the P + friction effort, never the D brake
   private double controlDt, turnP, turnD, turnFeedforward, turnRaw;
   private boolean turnClipped;
   private static final double TURN_SLEW_PER_S = 1.0;


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
               GoBildaPinpointDriver.EncoderDirection.FORWARD);
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
           sampleValid = next != null && finite(next.getX(DistanceUnit.METER))
                   && finite(next.getY(DistanceUnit.METER))
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
           angularVelocity = wrapDeg(getHeadingDeg() - anchorHeading) / dt;
           velocityValid = finite(velocityX) && finite(velocityY) && finite(angularVelocity);
           velocityTime = now;
           setVelocityAnchor(now);
       }
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
       return motionUsable() && Math.hypot(velocityX, velocityY) <= STILL_SPEED_MPS
               && Math.abs(angularVelocity) <= STILL_TURN_DPS;
   }


   public boolean isReady() { return referenceReady && !isResetting() && motionUsable(); }
   public boolean isResetting() { return resetState != ResetState.NONE; }
   public boolean canDriveManually() { return !isResetting() && !"CALIBRATING".equals(status); }


   public void processInputs(boolean up, boolean down, boolean left, boolean right,
                             boolean x, boolean b, boolean y) {
       if (isNavigating() || isResetting()) return;
       if (!isReady()) {
           if (up || down || left || right || x || b || y) lastResult = "NOT READY FOR NAVIGATION";
           return;
       }
       double forward = ((up ? 1 : 0) - (down ? 1 : 0)) * STEP_M;
       double robotLeft = ((left ? 1 : 0) - (right ? 1 : 0)) * STEP_M;
       double theta = Math.toRadians(getHeadingDeg());
       targetX += forward * Math.cos(theta) - robotLeft * Math.sin(theta);
       targetY += forward * Math.sin(theta) + robotLeft * Math.cos(theta);
       targetHeading = wrapDeg(targetHeading + ((x ? 1 : 0) - (b ? 1 : 0)) * HEADING_STEP_DEG);
       if (y) startNavigation();
   }


   public void startNavigation() {
       if (isNavigating()) return;
       if (drivetrain == null || !isReady() || !targetsFinite()) {
           lastResult = drivetrain == null ? "NO DRIVETRAIN" : "NOT READY / INVALID TARGET";
           stopDrive();
           return;
       }
       navState = NavState.TURNING;
       phaseStarted = lastControlTime = clock.seconds();
       settledSince = -1.0;
       commandX = commandY = 0.0;
       lastResult = "RUNNING";
       stopDrive();
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
       targetX = getX();
       targetY = getY();
       targetHeading = wrapDeg(getHeadingDeg());
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
       if (now - phaseStarted > TURN_TIMEOUT_S) {
           finish("TURN TIMEOUT"); // never translate after a failed turn
           return;
       }
       double error = wrapDeg(targetHeading - getHeadingDeg());
       if (Math.abs(error) <= TURN_TOLERANCE_DEG && isStationary()) {
           stopDrive();
           if (settledSince < 0) settledSince = now;
           if (now - settledSince >= TURN_SETTLE_S) {
               navState = NavState.DRIVING;
               phaseStarted = now;
               settledSince = -1.0;
               commandX = commandY = 0.0;
           }
           return;
       }
       settledSince = -1.0;
       drivetrain.drive(0, 0, TURN_DIRECTION * turnPower(error, TURN_KP, TURN_KD,
               TURN_MAX_POWER, dt), voltageScale);
   }


   private double turnPower(double error, double kp, double kd, double limit, double dt) {
       // The full normalized error drives P. Tolerance is used ONLY for settling.
       turnP = kp * error;
       // Deg/s from wrapped pose differences: no target-change derivative kick.
       turnD = -kd * angularVelocity;
       turnFeedforward = 0.0;
       double pd = turnP + turnD;
       if (Math.abs(error) > TURN_TOLERANCE_DEG && Math.abs(angularVelocity) < 8.0
               && pd * error > 0.0) {
           // Retained tapered friction model. This is NOT a guaranteed minimum
           // usable motor power: measure breakaway power before changing it.
           turnFeedforward = Math.copySign(
                   TURN_STATIC * Math.min(Math.abs(error) / 8.0, 1.0), error);
       }
       double desiredEffort = Range.clip(turnP + turnFeedforward, -limit, limit);
       // Remove obsolete effort immediately when the target correction reverses.
       if (desiredEffort * turnEffort < 0.0) turnEffort = 0.0;
       if (Math.abs(desiredEffort) <= Math.abs(turnEffort)) {
           turnEffort = desiredEffort; // do not delay reducing propulsion
       } else {
           double step = TURN_SLEW_PER_S * Math.min(dt, 0.05);
           turnEffort += Range.clip(desiredEffort - turnEffort, -step, step);
       }
       // Apply velocity damping AFTER the propulsion slew limiter so braking
       // can take effect on this control update, even while propulsion ramps.
       turnRaw = turnEffort + turnD;
       turnCommand = Range.clip(turnRaw, -limit, limit);
       turnClipped = Math.abs(turnRaw) > limit;
       return turnCommand;
   }

   private void handleDriving(double now, double dt, double voltageScale) {
       if (now - phaseStarted > DRIVE_TIMEOUT_S) {
           finish("DRIVE TIMEOUT");
           return;
       }
       double ex = targetX - getX(), ey = targetY - getY();
       double distance = Math.hypot(ex, ey);
       double headingError = wrapDeg(targetHeading - getHeadingDeg());
       boolean positionReached = distance <= DIST_TOLERANCE_M;
       if (positionReached && Math.abs(headingError) <= TURN_TOLERANCE_DEG && isStationary()) {
           stopDrive();
           commandX = commandY = 0.0;
           if (settledSince < 0) settledSince = now;
           if (now - settledSince >= ARRIVAL_SETTLE_S) finish("ARRIVED");
           return;
       }
       settledSince = -1.0;


       if (positionReached) {
           // No translational creep while correcting the final heading / waiting to settle.
           commandX = commandY = 0.0;
       } else {
           double speed = DIST_KP * distance
                   + DIST_STATIC * Math.min(distance / STATIC_RAMP_M, 1.0);
           // Signed damping can oppose travel to brake; no hard minimum power floor.
           double requestedX = speed * ex / distance - DIST_KD * velocityX;
           double requestedY = speed * ey / distance - DIST_KD * velocityY;
           double magnitude = Math.hypot(requestedX, requestedY);
           if (magnitude > DIST_MAX_POWER) {
               requestedX *= DIST_MAX_POWER / magnitude;
               requestedY *= DIST_MAX_POWER / magnitude;
           }
           double dx = requestedX - commandX, dy = requestedY - commandY;
           double delta = Math.hypot(dx, dy);
           double allowed = POWER_SLEW_PER_S * Math.min(dt, 0.05);
           double ratio = delta > allowed && delta > 0.0 ? allowed / delta : 1.0;
           commandX += dx * ratio;
           commandY += dy * ratio;
       }
       double theta = Math.toRadians(getHeadingDeg());
       double forward = commandX * Math.cos(theta) + commandY * Math.sin(theta);
       double left = -commandX * Math.sin(theta) + commandY * Math.cos(theta);
       double turn = turnPower(headingError, HOLD_KP, HOLD_KD, HOLD_MAX_POWER, dt);
       drivetrain.drive(forward, STRAFE_DIRECTION * left, TURN_DIRECTION * turn, voltageScale);
   }


   private void finish(String result) {
       navState = NavState.IDLE;
       lastResult = result;
       settledSince = -1.0;
       commandX = commandY = 0.0;
       stopDrive();
   }


   private void stopDrive() {
       turnCommand = turnEffort = 0.0;
       turnP = turnD = turnFeedforward = turnRaw = 0.0;
       turnClipped = false;
       if (drivetrain != null) drivetrain.drive(0, 0, 0, 1.0);
   }


   private boolean targetsFinite() {
       return finite(targetX) && finite(targetY) && finite(targetHeading);
   }


   // Programmatic adjustments stay field-relative; reject changes during motion/reset.
   public void adjustTargetX(double delta) {
       if (!isNavigating() && isReady() && finite(delta) && finite(targetX + delta)) targetX += delta;
   }
   public void adjustTargetY(double delta) {
       if (!isNavigating() && isReady() && finite(delta) && finite(targetY + delta)) targetY += delta;
   }
   public void adjustTargetHeading(double delta) {
       if (!isNavigating() && isReady() && finite(delta) && finite(targetHeading + delta))
           targetHeading = wrapDeg(targetHeading + delta);
   }
   public double getX() { return pose.getX(DistanceUnit.METER); }
   public double getY() { return pose.getY(DistanceUnit.METER); }
   public double getHeadingDeg() { return pose.getHeading(AngleUnit.DEGREES); }
   public double getTargetX() { return targetX; }
   public double getTargetY() { return targetY; }
   public double getTargetHeading() { return targetHeading; }
   public String getStatus() { return status; }
   public String getLastResult() { return lastResult; }
   /** Historical result of the last operation, not a live check of the current pose. */
   public boolean hasArrived() { return navState == NavState.IDLE && "ARRIVED".equals(lastResult); }
   public boolean isNavigating() { return navState != NavState.IDLE; }


   public void addTelemetry(Telemetry telemetry) {
       telemetry.addData("Pinpoint", status);
       telemetry.addData("Navigation ready", isReady());
       telemetry.addData("Pose (m / deg)", "X %.3f Y %.3f H %.1f", getX(), getY(), getHeadingDeg());
       telemetry.addData("Target (m / deg)", "X %.3f Y %.3f H %.1f", targetX, targetY, targetHeading);
       telemetry.addData("Error", "%.3f m / %.1f deg",
               Math.hypot(targetX - getX(), targetY - getY()), wrapDeg(targetHeading - getHeadingDeg()));
       telemetry.addData("Estimated motion", "%.3f m/s / %.1f deg/s (%s)",
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
       telemetry.addData("Hardware settings", "VERIFY offsets %.1f / %.1f mm; 4-bar; FWD/FWD",
               POD_X_OFFSET_MM, POD_Y_OFFSET_MM);
   }


   private static boolean finite(double value) { return !Double.isNaN(value) && !Double.isInfinite(value); }
   private static double wrapDeg(double degrees) { return AngleUnit.normalizeDegrees(degrees); }
}
