package org.firstinspires.ftc.teamcode;

/**
 * Shooting solution from up to three estimates (pure math, no FTC hardware; settings in ShotConfig):
 *
 *   1. HIVE TILT from the AprilTags. The hive is a seesaw: its tilt moves each cell's opening up/down and along the
 *      field, so the aim point is computed from a hive model (pivot + cell offset rotated by the tilt). When the tags
 *      are lost the last tilt is held for TILT_HOLD_S (hives only tip when scored), then TILT_DEFAULT_DEG is assumed.
 *   2. ODOMETRY: the robot's field pose (Pinpoint).
 *   3. APRILTAG POSE: the robot's field pose from a hive tag cluster of EITHER colour. The cluster's field spot comes
 *      from the hive model at that hive's tilt; the camera measures the cluster relative to the robot; with the
 *      Pinpoint heading that gives the robot's x, y (no Limelight field map needed).
 *
 * 2 and 3 are fused by inverse-variance weighting (each weighted by how much it is trusted). If they disagree by more
 * than FUSE_GATE_IN, one is wrong (wrong cell, odometry slip), so they are NOT averaged: the more trusted one is used
 * and a warning is raised. From the fused pose, the tilted aim point of OUR cell gives distance, height and aim
 * heading; flywheel RPM comes from RPM_TABLE and the launch angle from projectile physics (no drag) at that speed.
 *
 * Robot frame here: forward, LEFT, up (inches). Limelight robot space is forward, RIGHT, up: callers flip Y.
 */
public final class ShotSolver {
    private ShotSolver() { }
    static final double G_IN_S2 = 386.09;

    /** A robot FIELD pose estimate (x, y inches) with its 1-sigma error (in). */
    public static final class Estimate {
        public final boolean ok;
        public final double x, y, sigma;
        public final String source;
        Estimate(boolean ok, double x, double y, double sigma, String source) {
            this.ok = ok; this.x = x; this.y = y; this.sigma = sigma; this.source = source;
        }
        static Estimate none(String why) { return new Estimate(false, Double.NaN, Double.NaN, Double.NaN, why); }
    }

    public static final class Solution {
        public boolean ok;
        public String reason = "";
        public String warnings = "";
        public String used = "";             // which estimates went into the answer
        public double tiltDeg = Double.NaN;
        public String tiltSource = "";
        public int cellSide;                 // +1 = +Y cell, -1 = -Y cell
        public Estimate odo, tag, fused;
        public double distanceIn = Double.NaN, heightIn = Double.NaN;   // launcher exit -> aim point
        public double turnDeg = Double.NaN;  // turn this much (CCW +) so the launcher points at the aim point
        public double rpm = Double.NaN, exitSpeedInS = Double.NaN, angleDeg = Double.NaN;
        public boolean highArc;
    }

    /** Aim point (field x, y, z) of the cell on side `side` (+1 = +Y end) of our hive at tilt `tiltDeg`. */
    public static double[] aimPoint(boolean red, int side, double tiltDeg) {
        double t = Math.toRadians(tiltDeg);
        double along = side * ShotConfig.CELL_ALONG_IN, up = ShotConfig.CELL_AIM_ABOVE_PIVOT_IN;
        // Seesaw about the pivot (an axis along field X): rotating by +t raises the +Y end.
        double y = along * Math.cos(t) - up * Math.sin(t);
        double z = along * Math.sin(t) + up * Math.cos(t);
        return new double[]{red ? ShotConfig.RED_HIVE_X_IN : ShotConfig.BLUE_HIVE_X_IN, y,
                ShotConfig.HIVE_PIVOT_HEIGHT_IN + z};
    }

    /** Which side (+Y = +1) a tag cluster (0 red scoring, 1 red audience, 2 blue audience, 3 blue scoring) is on. */
    public static int sideOfCluster(int cluster) {
        boolean scoring = cluster == 0 || cluster == 3;
        int scoringSide = ShotConfig.SCORING_CELL_SIDE >= 0 ? 1 : -1;
        return scoring ? scoringSide : -scoringSide;
    }

    /** Field spot (x, y, z) of the tag cluster under a cell: the cell's aim point minus the tilted tag-to-opening offset. */
    public static double[] tagClusterField(int cluster, double tiltDeg) {
        boolean red = cluster == 0 || cluster == 1;
        double[] aim = aimPoint(red, sideOfCluster(cluster), tiltDeg);
        double t = Math.toRadians(tiltDeg), u = ShotConfig.TAG_TO_AIM_UP_IN;
        // The cell's "up" leans with the hive: field offset (0, -u sin t, u cos t).
        return new double[]{aim[0], aim[1] + u * Math.sin(t), aim[2] - u * Math.cos(t)};
    }

    /**
     * Robot field (x, y) from a tag cluster seen at (fwd, left) in the robot frame, the robot heading and that hive's
     * tilt: robot = cluster spot - (camera->cluster vector turned into the field frame). NaN if data is missing.
     */
    public static double[] robotPoseFromTag(int cluster, double tagFwd, double tagLeft, double headingDeg, double tiltDeg) {
        if (cluster < 0 || cluster > 3 || !finite(tagFwd) || !finite(tagLeft) || !finite(headingDeg) || !finite(tiltDeg)) {
            return new double[]{Double.NaN, Double.NaN};
        }
        double[] c = tagClusterField(cluster, tiltDeg);
        double h = Math.toRadians(headingDeg);
        return new double[]{c[0] - (tagFwd * Math.cos(h) - tagLeft * Math.sin(h)),
                            c[1] - (tagFwd * Math.sin(h) + tagLeft * Math.cos(h))};
    }

    /** Inverse-variance average of two pose estimates, or the more trusted one if they disagree (gate). */
    static Estimate fuse(Estimate a, Estimate b, StringBuilder warn) {
        if (!a.ok) return b;
        if (!b.ok) return a;
        double gap = Math.hypot(a.x - b.x, a.y - b.y);
        if (gap > ShotConfig.FUSE_GATE_IN) {
            Estimate keep = a.sigma <= b.sigma ? a : b;
            warn.append(String.format("%s and %s disagree by %.1f in, using %s; ", a.source, b.source, gap, keep.source));
            return keep;
        }
        double wa = 1 / sq(a.sigma), wb = 1 / sq(b.sigma), w = wa + wb;
        return new Estimate(true, (wa * a.x + wb * b.x) / w, (wa * a.y + wb * b.y) / w, Math.sqrt(1 / w),
                a.source + "+" + b.source);
    }

    /**
     * Full solution.
     * @param tiltDeg      tilt of OUR hive (NaN = unknown -> TILT_DEFAULT_DEG); tiltMeasured says if it came from tags
     * @param tagX,tagY    robot field pose from the tags (NaN if none); tagRangeIn camera->cluster distance
     * @param tagTiltKnown whether the hive the tags are on had a measured tilt (its pose is worse if not)
     */
    public static Solution solve(boolean red, double odoX, double odoY, double headingDeg,
                                 double tiltDeg, boolean tiltMeasured, String tiltSource,
                                 double tagX, double tagY, double tagRangeIn, boolean tagTiltKnown) {
        Solution s = new Solution();
        StringBuilder warn = new StringBuilder();
        // ---- Estimate 1: hive tilt (of our hive)
        if (!finite(tiltDeg)) {
            tiltDeg = ShotConfig.TILT_DEFAULT_DEG;
            tiltMeasured = false;
            tiltSource = "assumed";
            warn.append("hive tilt not measured; ");
        }
        s.tiltDeg = tiltDeg;
        s.tiltSource = tiltSource;

        // ---- Estimates 2 and 3: robot field pose
        s.odo = finite(odoX) && finite(odoY) ? new Estimate(true, odoX, odoY, ShotConfig.ODO_SIGMA_IN, "odometry")
                : Estimate.none("odometry not ready");
        s.tag = finite(tagX) && finite(tagY)
                ? new Estimate(true, tagX, tagY, ShotConfig.TAG_SIGMA_BASE_IN + ShotConfig.TAG_SIGMA_PER_IN
                    * (finite(tagRangeIn) ? tagRangeIn : 60) + (tagTiltKnown ? 0 : ShotConfig.TILT_ASSUMED_SIGMA_IN), "apriltag")
                : Estimate.none("no hive tag pose");
        s.fused = fuse(s.odo, s.tag, warn);
        if (!s.fused.ok || !finite(headingDeg)) {
            s.reason = "no pose (" + s.odo.source + ", " + s.tag.source + ")";
            s.warnings = warn.toString();
            return s;
        }
        s.used = "tilt(" + tiltSource + ") + " + s.fused.source;

        // ---- Our cell: the configured one, else the nearer end of our hive
        int side;
        if (ShotConfig.TARGET_CELL != 0) side = (ShotConfig.TARGET_CELL > 0) == (ShotConfig.SCORING_CELL_SIDE >= 0) ? 1 : -1;
        else side = s.fused.y < 0 ? -1 : 1;
        s.cellSide = side;
        double[] aim = aimPoint(red, side, tiltDeg);

        // ---- Geometry from the launcher exit (robot frame: forward, left)
        double h = Math.toRadians(headingDeg), dx = aim[0] - s.fused.x, dy = aim[1] - s.fused.y;
        double f = dx * Math.cos(h) + dy * Math.sin(h) - ShotConfig.LAUNCHER_FWD_IN;
        double l = -dx * Math.sin(h) + dy * Math.cos(h) - ShotConfig.LAUNCHER_LEFT_IN;
        s.distanceIn = Math.hypot(f, l);
        s.heightIn = aim[2] - ShotConfig.LAUNCHER_HEIGHT_IN;
        s.turnDeg = Math.toDegrees(Math.atan2(l, f));

        // ---- Flywheel and angle
        s.rpm = rpmFor(s.distanceIn);
        s.exitSpeedInS = ShotConfig.SPEED_EFFICIENCY * (s.rpm / 60.0) * 2 * Math.PI * ShotConfig.FLYWHEEL_RADIUS_IN;
        s.highArc = ShotConfig.HIGH_ARC;
        double ang = launchAngleDeg(s.distanceIn, s.heightIn, s.exitSpeedInS, s.highArc);
        if (!inHood(ang)) {
            double other = launchAngleDeg(s.distanceIn, s.heightIn, s.exitSpeedInS, !s.highArc);
            if (inHood(other)) { ang = other; s.highArc = !s.highArc; warn.append("used the other arc; "); }
        }
        s.warnings = warn.toString();
        if (Double.isNaN(ang)) { s.reason = "out of range at " + Math.round(s.rpm) + " rpm (raise RPM_TABLE)"; return s; }
        if (!inHood(ang)) { s.reason = String.format("needs %.1f deg, hood range %.0f-%.0f", ang,
                ShotConfig.HOOD_MIN_DEG, ShotConfig.HOOD_MAX_DEG); s.angleDeg = ang; return s; }
        s.angleDeg = ang;
        s.ok = true;
        return s;
    }

    /** tan(theta) = (v^2 +- sqrt(v^4 - g(g x^2 + 2 y v^2))) / (g x). NaN if unreachable. */
    public static double launchAngleDeg(double x, double y, double v, boolean highArc) {
        if (!(x > 0) || !(v > 0)) return Double.NaN;
        double v2 = v * v, disc = v2 * v2 - G_IN_S2 * (G_IN_S2 * x * x + 2 * y * v2);
        if (disc < 0) return Double.NaN;
        return Math.toDegrees(Math.atan((v2 + (highArc ? 1 : -1) * Math.sqrt(disc)) / (G_IN_S2 * x)));
    }

    /** Linear interpolation in RPM_TABLE, clamped at both ends. */
    public static double rpmFor(double d) {
        double[][] t = ShotConfig.RPM_TABLE;
        if (d <= t[0][0]) return t[0][1];
        for (int i = 1; i < t.length; i++) {
            if (d <= t[i][0]) return t[i - 1][1] + (d - t[i - 1][0]) / (t[i][0] - t[i - 1][0]) * (t[i][1] - t[i - 1][1]);
        }
        return t[t.length - 1][1];
    }

    /** Hood servo position for a launch angle (two-point linear calibration). */
    public static double hoodPosition(double angleDeg) {
        double f = (angleDeg - ShotConfig.HOOD_ANGLE_A_DEG) / (ShotConfig.HOOD_ANGLE_B_DEG - ShotConfig.HOOD_ANGLE_A_DEG);
        double pos = ShotConfig.HOOD_POS_A + f * (ShotConfig.HOOD_POS_B - ShotConfig.HOOD_POS_A);
        return Math.max(0.0, Math.min(1.0, pos));
    }

    static boolean inHood(double a) { return finite(a) && a >= ShotConfig.HOOD_MIN_DEG && a <= ShotConfig.HOOD_MAX_DEG; }
    private static double sq(double v) { return v * v; }
    private static boolean finite(double v) { return !Double.isNaN(v) && !Double.isInfinite(v); }
}
