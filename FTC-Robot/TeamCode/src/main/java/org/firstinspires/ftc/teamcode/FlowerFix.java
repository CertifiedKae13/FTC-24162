package org.firstinspires.ftc.teamcode;

/**
 * Absolute position from one FLOWER sighting, no AprilTags. Pure math (no FTC classes), so it can be tested off-robot.
 *
 * The flowers are fixed to the perimeter walls at known field positions. The slot-1 SnapScript finds a flower's two
 * green posts and reports:
 *   tx          bearing to the flower centre (deg, + = flower to the RIGHT of the crosshair), and
 *   spacingPx   centre-to-centre distance between the two posts in the image (px).
 * Posts look smaller the farther away they are, so   depth = K / spacingPx,   where K = focal length (px) x real post
 * spacing (in). K is measured once with FlowerRangeCal. Heading comes from the Pinpoint IMU (it barely drifts), so
 * one sighting is enough for x and y:   camera = flower - (vector from camera to flower);   robot = camera - mount offset.
 *
 * Field frame as FieldStart: inches, origin at the centre, +X toward blue, heading 0 = +X, CCW positive.
 */
public final class FlowerFix {
    private FlowerFix() { }

    public static final class Result {
        public final boolean ok;
        public final double x, y;           // robot centre, field inches
        public final double rangeIn;        // camera to flower centre, along the line of sight
        public final double viewAngleDeg;   // how far off head-on we view the flower (0 = square to it)
        public final String reason;         // why not ok
        Result(boolean ok, double x, double y, double rangeIn, double viewAngleDeg, String reason) {
            this.ok = ok; this.x = x; this.y = y; this.rangeIn = rangeIn; this.viewAngleDeg = viewAngleDeg;
            this.reason = reason;
        }
        static Result fail(String why) { return new Result(false, Double.NaN, Double.NaN, Double.NaN, Double.NaN, why); }
    }

    /**
     * @param flowerX,flowerY   flower centre (field in)
     * @param flowerFacingDeg   direction the flower's front faces, into the field (field deg)
     * @param headingDeg        robot heading from odometry (field deg, CCW +)
     * @param txDeg             Limelight tx to the flower centre (+ = right)
     * @param spacingPx         post spacing in the image (px), from llpython[4]; must come from a post PAIR
     * @param k                 range constant: depth_in x spacing_px for a head-on view (FlowerRangeCal)
     * @param camForwardIn      camera lens ahead of the robot centre (in)
     * @param camLeftIn         camera lens left of the robot centre (in)
     * @param maxViewDeg        reject views more oblique than this (spacing shrinks, range gets unreliable)
     */
    public static Result solve(double flowerX, double flowerY, double flowerFacingDeg,
                               double headingDeg, double txDeg, double spacingPx, double k,
                               double camForwardIn, double camLeftIn, double maxViewDeg) {
        if (!finite(flowerX) || !finite(flowerY) || !finite(flowerFacingDeg) || !finite(headingDeg)
                || !finite(txDeg) || !finite(spacingPx) || !finite(k) || !finite(camForwardIn) || !finite(camLeftIn)) {
            return Result.fail("missing data");
        }
        if (spacingPx < 2.0) return Result.fail("posts too close together in image");
        if (k <= 0) return Result.fail("range constant K not set (run Flower Range Cal)");

        // 1) Line of sight, camera -> flower, in field degrees. tx is + to the RIGHT = clockwise = negative.
        double sightDeg = headingDeg - txDeg;

        // 2) View angle: between the flower's facing and the direction from the flower back to the camera.
        //    Head-on = 0. The posts stand side by side along the wall, so seen at angle a their spacing shrinks
        //    by cos(a): correct the range for it, and refuse very oblique views.
        double viewDeg = wrap(sightDeg + 180.0 - flowerFacingDeg);
        if (Math.abs(viewDeg) > maxViewDeg) return Result.fail(String.format("view too oblique (%.0f deg)", viewDeg));
        double cosView = Math.cos(Math.toRadians(viewDeg));

        // 3) Depth along the camera's optical axis (pixel size scales with depth, not slant range), then the
        //    sideways offset from tx. Camera frame: +forward, +left; tx + = right -> left = -depth*tan(tx).
        double depth = k * cosView / spacingPx;
        double tx = Math.toRadians(txDeg);
        double camFwdToFlower = depth;
        double camLeftToFlower = -depth * Math.tan(tx);
        double range = depth / Math.cos(tx);

        // 4) Rotate camera -> flower into the field frame and step back from the flower to the camera.
        double h = Math.toRadians(headingDeg);
        double c = Math.cos(h), s = Math.sin(h);
        double camX = flowerX - (camFwdToFlower * c - camLeftToFlower * s);
        double camY = flowerY - (camFwdToFlower * s + camLeftToFlower * c);

        // 5) Camera mount offset -> robot centre.
        double x = camX - (camForwardIn * c - camLeftIn * s);
        double y = camY - (camForwardIn * s + camLeftIn * c);
        if (!finite(x) || !finite(y)) return Result.fail("math overflow");
        return new Result(true, x, y, range, viewDeg, "OK");
    }

    /** Index of the flower whose expected bearing from the predicted pose best matches tx (they all look alike). */
    public static int pickFlower(double[][] flowers, double predX, double predY, double headingDeg, double txDeg) {
        int best = -1;
        double bestErr = Double.POSITIVE_INFINITY;
        for (int i = 0; i < flowers.length; i++) {
            double bearing = Math.toDegrees(Math.atan2(flowers[i][1] - predY, flowers[i][0] - predX));
            double err = Math.abs(wrap(bearing - (headingDeg - txDeg)));
            if (err < bestErr) { bestErr = err; best = i; }
        }
        return best;
    }

    static double wrap(double deg) {
        double w = deg % 360.0;
        if (w >= 180.0) w -= 360.0;
        if (w < -180.0) w += 360.0;
        return w;
    }

    private static boolean finite(double v) { return !Double.isNaN(v) && !Double.isInfinite(v); }
}
