package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/**
 * Reads the Limelight 3A's AprilTag pipeline every loop (call update()) and exposes two values to the rest of the
 * code: the tilt angle of a hive (ours when its tags are in view, else the other one; tags 30-45 of both colours
 * count) and the robot's pose on the field (Limelight botpose, or from the hive tags + heading via update(heading)). It only reads; it never drives or resets the
 * odometry. Uses the FTC SDK's built-in Limelight3A (not NetworkTables / LimelightLib, which are FRC-side).
 *
 * "No valid target" is explicit: Reading.valid is false, the values are NaN (never stale numbers or zeros), and
 * Reading.status says why (not configured, not connected, stale, wrong pipeline, no tag).
 *
 * Hive tilt: one of the tag's angles in robot space (assumes the robot is level). Limelight documents robot space
 * (X forward, Y right, Z up) but NOT which angle is which or its sign, so calibrate it: look at the ll_raw_* graph
 * values while tilting the hive, pick the angle that moves (LL_TILT_AXIS), set LL_TILT_ZERO_DEG so a known hive state
 * reads 0, and flip LL_TILT_SIGN if it reads backwards.
 *
 * Pose: the Limelight's botpose (MegaTag 1), in meters in the field map's frame, converted to inches and rotated by
 * LL_FIELD_ROTATION_DEG into this project's field frame (center origin, +X toward blue, heading 0 = +X). The pose is
 * only as good as the field map uploaded to the Limelight: it must be a BIOBUZZ map with the real tag positions.
 */
public final class LimelightVision {
    private static final String TAG = "LimelightVision";
    /** Name of the Limelight in the Robot Controller hardware configuration. */
    public static final String DEVICE_NAME = "limelight";
    /** Shown when the active robot configuration has no Limelight 3A at all. */
    static final String NOT_CONFIGURED = "NOT CONFIGURED: add Limelight 3A (Ethernet Device) named '" + DEVICE_NAME
            + "' to the ACTIVE robot config";
    // ===== FILL IN / CHECK: AprilTag ids on each hive. Prefilled from the FTC SDK's BIOBUZZ tag library and manual
    // Figure 9-17 (red scoring 30-33, red audience 34-37, blue audience 38-41, blue scoring 42-45); confirm on the field.
    public static final int[] RED_HIVE_TAG_IDS = {30, 31, 32, 33, 34, 35, 36, 37};
    public static final int[] BLUE_HIVE_TAG_IDS = {38, 39, 40, 41, 42, 43, 44, 45};

    /** One snapshot of what the camera sees. Values are NaN when not valid. */
    public static final class Reading {
        public final boolean valid;          // true only when a fresh, usable AprilTag result is present
        public final String status;          // "OK" or the reason there is no valid target
        public final boolean hasHiveTilt;    // a tag of our hive was seen
        public final double hiveTiltDeg;     // NaN unless hasHiveTilt
        public final boolean hasPose;        // the Limelight produced a field pose from the tags
        public final double x, y, headingDeg;// robot pose, inches / degrees [0,360); NaN unless hasPose
        public final int tagCount;
        public final long stalenessMs;
        /** Raw angles of our hive's tag in robot space (deg), for calibrating the tilt axis; NaN if no hive tag. */
        public final double rawPitch, rawYaw, rawRoll;
        /** Id of the first hive tag of ours used for the tilt, or -1. */
        public int hiveTagId = -1;
        /**
         * Centre of the tag cluster of hiveTagId's cell, in Limelight ROBOT space converted to inches (X forward,
         * Y right, Z up; needs the camera pose set in the Limelight UI). NaN if no tag of ours. tagCluster = 0..3:
         * (id - 30) / 4 -> 0 red scoring, 1 red audience, 2 blue audience, 3 blue scoring; -1 if none.
         */
        public double tagFwdIn = Double.NaN, tagRightIn = Double.NaN, tagUpIn = Double.NaN;
        public int tagCluster = -1;
        /** Which hive the tilt / tag position came from (our alliance's when its tags are in view). */
        public boolean hiveIsRed;
        /** "botpose" (Limelight field map), "tags" (hive model + Pinpoint heading) or "" when hasPose is false. */
        public String poseSource = "";

        Reading(boolean valid, String status, boolean hasHiveTilt, double hiveTiltDeg, boolean hasPose,
                double x, double y, double headingDeg, int tagCount, long stalenessMs,
                double rawPitch, double rawYaw, double rawRoll) {
            this.valid = valid; this.status = status; this.hasHiveTilt = hasHiveTilt; this.hiveTiltDeg = hiveTiltDeg;
            this.hasPose = hasPose; this.x = x; this.y = y; this.headingDeg = headingDeg;
            this.tagCount = tagCount; this.stalenessMs = stalenessMs;
            this.rawPitch = rawPitch; this.rawYaw = rawYaw; this.rawRoll = rawRoll;
        }

        static Reading none(String why) {
            return new Reading(false, why, false, Double.NaN, false, Double.NaN, Double.NaN, Double.NaN, 0, -1,
                    Double.NaN, Double.NaN, Double.NaN);
        }
    }

    private Limelight3A limelight;
    private boolean redAlliance = true;
    private Reading latest = Reading.none("NOT STARTED");
    // Distance to whatever the CURRENT pipeline targets (e.g. a pollen colour pipeline), from ty: no AprilTags needed.
    private double targetDistanceIn = Double.NaN;
    private double targetTxDeg = Double.NaN;

    /** Looks the Limelight up in the hardware map. If it is not configured, every reading is "NOT CONFIGURED". */
    public LimelightVision(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.tryGet(Limelight3A.class, DEVICE_NAME);
            if (limelight == null) {
                // Named differently in the robot config (e.g. "limelight3a")? Use the only/first Limelight there is.
                java.util.List<Limelight3A> all = hardwareMap.getAll(Limelight3A.class);
                if (!all.isEmpty()) {
                    limelight = all.get(0);
                    Log.w(TAG, "no device named '" + DEVICE_NAME + "', using " + hardwareMap.getNamesOf(limelight));
                }
            }
            if (limelight != null) {
                limelight.setPollRateHz((int) AutoConfig.LL_POLL_HZ);
                limelight.pipelineSwitch((int) AutoConfig.LL_PIPELINE);
                limelight.start();
            } else {
                latest = Reading.none(NOT_CONFIGURED);
            }
        } catch (RuntimeException e) {
            Log.w(TAG, "limelight unavailable: " + e);
            limelight = null;
            latest = Reading.none(NOT_CONFIGURED);
        }
    }

    /** Which hive is "ours": the red alliance's tags are 30-37, the blue alliance's 38-45. */
    public void setAlliance(boolean red) { redAlliance = red; }

    public boolean isAvailable() { return limelight != null; }

    /** True if the latest update() saw a tag of OUR hive, so getHiveTiltDeg() is a real, fresh angle of it. */
    public boolean hasHiveTilt() { return latest.valid && latest.hasHiveTilt && latest.hiveIsRed == redAlliance; }

    /** Hive tilt in degrees from the latest update(), or NaN when there is no valid reading (never a stale angle). */
    public double getHiveTiltDeg() { return hasHiveTilt() ? latest.hiveTiltDeg : Double.NaN; }

    /** Id of the first hive tag of ours in the latest frame, or -1 if none. */
    public int getHiveTagId() { return hasHiveTilt() ? latest.hiveTagId : -1; }

    /** The newest reading from update(). Never stale: a missing or old result is reported as not valid. */
    public Reading latest() { return latest; }

    /** Poll the camera. Call once per loop; returns (and stores) the new reading. Never throws. */
    public Reading update() {
        latest = read();
        return latest;
    }

    /**
     * Like update(), and also gives the robot's FIELD pose (x, y) from the hive tags when the Limelight has no field
     * map: the tag cluster's spot comes from the hive model (ShotConfig), its offset from the camera, and the robot
     * heading passed here (Pinpoint; NaN = no tag pose).
     */
    public Reading update(double headingDeg) {
        poseHeadingDeg = headingDeg;
        try { return update(); } finally { poseHeadingDeg = Double.NaN; }
    }

    private double poseHeadingDeg = Double.NaN;

    /**
     * Ground distance (inches) from the camera to the target of the current pipeline, or NaN if no fresh target.
     * Uses only the target's vertical angle ty and the camera geometry, so it needs NO AprilTags: point a colour
     * pipeline at pollen and this is the distance to the ball. Fill in LL_CAMERA_HEIGHT_IN, LL_CAMERA_DOWN_DEG and
     * LL_TARGET_HEIGHT_IN (AutoConfig); until they are set it returns NaN.
     */
    public double getTargetDistanceIn() { return targetDistanceIn; }

    /** Horizontal angle (deg, + = right) to the current pipeline's target, or NaN if none. */
    public double getTargetTxDeg() { return targetTxDeg; }

    public boolean hasTargetDistance() { return !Double.isNaN(targetDistanceIn); }

    /** Result of looking for a target on one pipeline (e.g. the green flower colour pipeline). */
    public static final class Sighting {
        public final boolean seen;
        public final double txDeg, tyDeg, areaPct;
        public final String status;
        /** Limelight frame time (s), to tell new frames from repeats. NaN if none. */
        public final double frameStamp;
        /** Flower SnapScript llpython[0]: 1 = post pair, 0.5 = single post, NaN = no python output. */
        public final double pythonFound;
        /** Flower SnapScript llpython[4]: post spacing in px (pair only), NaN otherwise. Used for range. */
        public final double spacingPx;
        Sighting(boolean seen, double tx, double ty, double ta, String status) {
            this(seen, tx, ty, ta, status, Double.NaN, Double.NaN, Double.NaN);
        }
        Sighting(boolean seen, double tx, double ty, double ta, String status, double frameStamp, double pythonFound,
                 double spacingPx) {
            this.seen = seen; this.txDeg = tx; this.tyDeg = ty; this.areaPct = ta; this.status = status;
            this.frameStamp = frameStamp; this.pythonFound = pythonFound; this.spacingPx = spacingPx;
        }
    }

    /**
     * Is a target of `pipeline` in view right now? Only a fresh result FROM THAT PIPELINE counts (the frame or two
     * after a pipeline switch still come from the old one and are ignored). Never throws.
     */
    public Sighting look(int pipeline) {
        if (limelight == null) return new Sighting(false, Double.NaN, Double.NaN, Double.NaN, NOT_CONFIGURED);
        try {
            if (!limelight.isConnected()) return new Sighting(false, Double.NaN, Double.NaN, Double.NaN, "NOT CONNECTED");
            LLResult r = limelight.getLatestResult();
            if (r == null) return new Sighting(false, Double.NaN, Double.NaN, Double.NaN, "NO DATA");
            if (r.getPipelineIndex() != pipeline) return new Sighting(false, Double.NaN, Double.NaN, Double.NaN, "SWITCHING PIPELINE");
            if (r.getStaleness() > AutoConfig.LL_MAX_STALENESS_MS) return new Sighting(false, Double.NaN, Double.NaN, Double.NaN, "STALE");
            if (!r.isValid()) return new Sighting(false, Double.NaN, Double.NaN, Double.NaN, "NOT IN VIEW");
            double[] py = r.getPythonOutput();
            double found = py != null && py.length > 0 ? py[0] : Double.NaN;
            double spacing = found >= 1.0 && py.length > 4 && py[4] > 0 ? py[4] : Double.NaN;
            return new Sighting(true, r.getTx(), r.getTy(), r.getTa(), "SEEN", r.getTimestamp(), found, spacing);
        } catch (RuntimeException e) {
            return new Sighting(false, Double.NaN, Double.NaN, Double.NaN, "READ ERROR");
        }
    }

    /**
     * The flower decision, shared by FlowerAuto (lookForSteady) and Flower Vision Test: from the fresh frames that saw
     * a target, need LL_STEADY_MIN_FRAMES, then take median tx / ty / ta, and median post spacing only if enough of
     * them saw the post PAIR. `last` supplies the reason when there were too few frames.
     */
    public static Sighting summarize(List<Sighting> got, Sighting last) {
        if (got.size() < (int) AutoConfig.LL_STEADY_MIN_FRAMES) {
            return new Sighting(false, Double.NaN, Double.NaN, Double.NaN,
                    "only " + got.size() + " frames (" + (last == null ? "-" : last.status) + ")");
        }
        double[] tx = new double[got.size()], ty = new double[got.size()], ta = new double[got.size()];
        java.util.ArrayList<Double> spacing = new java.util.ArrayList<>();
        int pairs = 0;
        for (int i = 0; i < got.size(); i++) {
            tx[i] = got.get(i).txDeg; ty[i] = got.get(i).tyDeg; ta[i] = got.get(i).areaPct;
            if (got.get(i).pythonFound >= 1.0) pairs++;
            if (!Double.isNaN(got.get(i).spacingPx)) spacing.add(got.get(i).spacingPx);
        }
        // Range needs the post PAIR: only trust spacing if enough frames saw both posts.
        double sp = Double.NaN;
        if (spacing.size() >= (int) AutoConfig.LL_STEADY_MIN_FRAMES) {
            double[] v = new double[spacing.size()];
            for (int i = 0; i < v.length; i++) v[i] = spacing.get(i);
            sp = median(v);
        }
        return new Sighting(true, median(tx), median(ty), median(ta),
                "SEEN " + got.size() + " frames, " + pairs + " pair", Double.NaN, Double.NaN, sp);
    }

    private static double median(double[] v) {
        double[] s = v.clone();
        java.util.Arrays.sort(s);
        int n = s.length;
        return n % 2 == 1 ? s[n / 2] : 0.5 * (s[n / 2 - 1] + s[n / 2]);
    }

    /** Raw facts about the latest Limelight result, for diagnostics (which pipeline really ran, valid flag, ...). */
    public String debugLine() {
        if (limelight == null) return NOT_CONFIGURED;
        try {
            if (!limelight.isConnected()) return "NOT CONNECTED";
            LLResult r = limelight.getLatestResult();
            if (r == null) return "NO DATA (no result yet)";
            double[] py = r.getPythonOutput();
            StringBuilder p = new StringBuilder();
            if (py == null || py.length == 0) p.append("none");
            else for (int i = 0; i < Math.min(6, py.length); i++) p.append(i == 0 ? "" : ",").append(String.format("%.1f", py[i]));
            return String.format("pipe %d  valid %b  stale %d ms  tx %.1f  tags %d  py [%s]",
                    r.getPipelineIndex(), r.isValid(), r.getStaleness(), r.getTx(),
                    r.getFiducialResults() == null ? 0 : r.getFiducialResults().size(), p);
        } catch (RuntimeException e) {
            return "READ ERROR " + e.getClass().getSimpleName();
        }
    }

    /** Switch the Limelight to another pipeline slot (e.g. a pollen colour pipeline for distance). */
    public void usePipeline(int slot) {
        try { if (limelight != null) limelight.pipelineSwitch(slot); } catch (RuntimeException ignored) { }
    }

    private void updateTargetDistance(LLResult r) {
        if (!r.isValid()) { clearTarget(); return; }
        // Camera tilted down by LL_CAMERA_DOWN_DEG; target is ty degrees above (+) / below (-) the crosshair.
        double down = Math.toRadians(AutoConfig.LL_CAMERA_DOWN_DEG - r.getTy());
        double dh = AutoConfig.LL_CAMERA_HEIGHT_IN - AutoConfig.LL_TARGET_HEIGHT_IN;
        if (Double.isNaN(down) || Double.isNaN(dh) || down <= Math.toRadians(1) || dh <= 0) { clearTarget(); return; }
        targetDistanceIn = dh / Math.tan(down);
        targetTxDeg = r.getTx();
    }

    private void clearTarget() { targetDistanceIn = Double.NaN; targetTxDeg = Double.NaN; }

    public void close() {
        try { if (limelight != null) limelight.stop(); } catch (RuntimeException ignored) { }
    }

    private Reading read() {
        if (limelight == null) return Reading.none(NOT_CONFIGURED);
        try {
            if (!limelight.isConnected()) { clearTarget(); return Reading.none("LIMELIGHT NOT CONNECTED"); }
            LLResult r = limelight.getLatestResult();
            if (r == null) { clearTarget(); return Reading.none("NO DATA"); }
            if (r.getStaleness() > AutoConfig.LL_MAX_STALENESS_MS) { clearTarget(); return Reading.none("STALE"); }
            updateTargetDistance(r);                 // works on any pipeline (colour, retroreflective, AprilTag)
            if (r.getPipelineIndex() != (int) AutoConfig.LL_PIPELINE) return Reading.none("WRONG PIPELINE");
            // Each rejection says which check failed (they used to all read "NO VALID TARGET").
            if (!r.isValid()) return Reading.none("NO VALID TARGET: Limelight reports no target (tv=0, check its ID filter)");
            List<LLResultTypes.FiducialResult> tags = r.getFiducialResults();
            if (tags == null || tags.isEmpty()) return Reading.none("NO VALID TARGET: target but no AprilTag data (pipeline type?)");

            // Hive tags of BOTH colours count (30-45). If tags of our alliance's hive are in view, that hive is used;
            // otherwise the other one. Tilt and position come only from the chosen hive's tags.
            boolean anyOurs = false, anyHive = false;
            for (LLResultTypes.FiducialResult t : tags) {
                if (!isHiveTag(t.getFiducialId()) || t.getTargetPoseRobotSpace() == null) continue;
                anyHive = true;
                if (isOurs(t.getFiducialId())) anyOurs = true;
            }
            boolean useRed = anyOurs ? redAlliance : !redAlliance;

            double tiltSum = 0, pitchSum = 0, yawSum = 0, rollSum = 0;
            int tiltCount = 0, firstId = -1;
            double fwdSum = 0, rightSum = 0, upSum = 0;
            int posCount = 0;
            for (LLResultTypes.FiducialResult t : tags) {
                int id = t.getFiducialId();
                if (!isHiveTag(id) || isRedHive(id) != useRed) continue;
                Pose3D p = t.getTargetPoseRobotSpace();
                if (p == null) continue;
                double pitch = p.getOrientation().getPitch(AngleUnit.DEGREES);
                double yaw = p.getOrientation().getYaw(AngleUnit.DEGREES);
                double roll = p.getOrientation().getRoll(AngleUnit.DEGREES);
                // Limelight documents the axes of robot space but not the sign or axis of each tag angle, so which
                // angle is the hive tilt is a setting (LL_TILT_AXIS: 0 pitch, 1 yaw, 2 roll) found by calibration.
                int axis = (int) AutoConfig.LL_TILT_AXIS;
                double chosen = axis == 1 ? yaw : axis == 2 ? roll : pitch;
                if (Double.isNaN(chosen)) continue;
                tiltSum += AutoConfig.LL_TILT_SIGN * (chosen - AutoConfig.LL_TILT_ZERO_DEG);
                pitchSum += pitch; yawSum += yaw; rollSum += roll;
                if (firstId < 0) firstId = id;
                tiltCount++;
                // Average the positions of the tags under the SAME cell as the first one (4 tags per cell).
                if ((id - 30) / 4 == (firstId - 30) / 4) {
                    Position tp = p.getPosition().toUnit(DistanceUnit.INCH);
                    fwdSum += tp.x; rightSum += tp.y; upSum += tp.z;
                    posCount++;
                }
            }
            boolean hasTilt = tiltCount > 0;
            double tilt = hasTilt ? tiltSum / tiltCount : Double.NaN;
            double rp = hasTilt ? pitchSum / tiltCount : Double.NaN;
            double ry = hasTilt ? yawSum / tiltCount : Double.NaN;
            double rr = hasTilt ? rollSum / tiltCount : Double.NaN;

            boolean hasPose = false;
            String poseSource = "";
            double x = Double.NaN, y = Double.NaN, h = Double.NaN;
            Pose3D bot = r.getBotpose();
            if (bot != null && r.getBotposeTagCount() > 0) {
                Position pos = bot.getPosition().toUnit(DistanceUnit.INCH);
                double yaw = bot.getOrientation().getYaw(AngleUnit.DEGREES);
                double rot = Math.toRadians(AutoConfig.LL_FIELD_ROTATION_DEG);
                double fx = pos.x * Math.cos(rot) - pos.y * Math.sin(rot);
                double fy = pos.x * Math.sin(rot) + pos.y * Math.cos(rot);
                // A botpose of exactly (0,0,0) means the Limelight had no solution, not "robot at center".
                if (!(pos.x == 0 && pos.y == 0 && yaw == 0) && !Double.isNaN(fx) && !Double.isNaN(fy) && !Double.isNaN(yaw)) {
                    hasPose = true;
                    poseSource = "botpose";
                    x = fx;
                    y = fy;
                    h = (((yaw + AutoConfig.LL_FIELD_ROTATION_DEG) % 360) + 360) % 360;
                }
            }
            // No field map: field pose from the tag cluster's known spot on the hive model + the Pinpoint heading.
            int cluster = posCount > 0 ? (firstId - 30) / 4 : -1;
            if (!hasPose && cluster >= 0 && !Double.isNaN(poseHeadingDeg)) {
                double[] xy = ShotSolver.robotPoseFromTag(cluster, fwdSum / posCount, -rightSum / posCount,
                        poseHeadingDeg, hasTilt ? tilt : ShotConfig.TILT_DEFAULT_DEG);
                if (!Double.isNaN(xy[0]) && !Double.isNaN(xy[1])) {
                    hasPose = true;
                    poseSource = "tags";
                    x = xy[0];
                    y = xy[1];
                    h = ((poseHeadingDeg % 360) + 360) % 360;
                }
            }
            if (!hasTilt && !hasPose) {
                StringBuilder ids = new StringBuilder();
                for (LLResultTypes.FiducialResult t : tags) ids.append(ids.length() == 0 ? "" : ",").append(t.getFiducialId());
                return Reading.none("NO VALID TARGET: saw tag " + ids + (anyHive ? ", no 3D pose for it"
                        : ", not a hive tag (30-45)") + " and no field pose");
            }
            Reading out = new Reading(true, "OK", hasTilt, tilt, hasPose, x, y, h, tags.size(), r.getStaleness(), rp, ry, rr);
            out.hiveTagId = firstId;
            out.hiveIsRed = useRed;
            out.poseSource = poseSource;
            if (posCount > 0) {
                out.tagFwdIn = fwdSum / posCount;
                out.tagRightIn = rightSum / posCount;
                out.tagUpIn = upSum / posCount;
                out.tagCluster = cluster;
            }
            return out;
        } catch (RuntimeException e) {
            Log.w(TAG, "read failed: " + e);
            return Reading.none("READ ERROR");
        }
    }

    private static boolean isHiveTag(int id) { return id >= 30 && id <= 45; }
    private static boolean isRedHive(int id) { return id >= 30 && id <= 37; }

    private boolean isOurs(int id) {
        for (int ours : redAlliance ? RED_HIVE_TAG_IDS : BLUE_HIVE_TAG_IDS) if (ours == id) return true;
        return false;
    }
}
