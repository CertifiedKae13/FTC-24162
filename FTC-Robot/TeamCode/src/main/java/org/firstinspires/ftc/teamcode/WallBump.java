package org.firstinspires.ftc.teamcode;

/**
 * Camera-free absolute position fix: when the robot is pushed flat against a field wall, its position
 * along that wall's axis and its heading are known exactly, whatever the odometry had drifted to.
 * Pure math (no FTC classes). Field frame as FieldStart: inches, origin center, +X toward blue.
 *
 *   wall      outward normal   coordinate that becomes known
 *   PLUS_X    heading 0        x =  wallHalf - (half the robot's size on the face that touches)
 *   MINUS_X   heading 180      x = -wallHalf + ...
 *   PLUS_Y    heading 90       y =  wallHalf - ...
 *   MINUS_Y   heading 270      y = -wallHalf + ...
 *
 * Only that one coordinate and the heading are corrected; the other coordinate keeps its (drifting)
 * value until a bump on a perpendicular wall fixes it.
 */
public final class WallBump {
    private WallBump() {}

    public enum Wall {
        PLUS_X(1, 0, 0), MINUS_X(-1, 0, 180), PLUS_Y(0, 1, 90), MINUS_Y(0, -1, 270);
        final int nx, ny;
        final double angleDeg;
        Wall(int nx, int ny, double angleDeg) { this.nx = nx; this.ny = ny; this.angleDeg = angleDeg; }
    }

    public static final class Result {
        public final boolean ok;
        public final double x, y, headingDeg;
        public final String reason;
        Result(boolean ok, double x, double y, double headingDeg, String reason) {
            this.ok = ok; this.x = x; this.y = y; this.headingDeg = headingDeg; this.reason = reason;
        }
    }

    private static double wrap180(double d) {
        d = ((d % 360) + 540) % 360 - 180;
        return d;
    }

    /** Which quarter-turn face points at the wall: k = round(rel/90); NaN skew means not square enough. */
    private static double rel(Wall w, double headingDeg) { return wrap180(w.angleDeg - headingDeg); }

    /** Half-size of the robot on the face that would touch the wall, or NaN if the heading is too skewed. */
    public static double faceHalf(Wall w, double headingDeg, double halfLength, double halfWidth, double maxSkewDeg) {
        double r = rel(w, headingDeg);
        long k = Math.round(r / 90.0);
        if (Math.abs(r - 90.0 * k) > maxSkewDeg) return Double.NaN;
        return (k % 2 == 0) ? halfLength : halfWidth;
    }

    /** Where to aim so the robot presses into the wall: the contact coordinate pushed `overdriveIn` further in. */
    public static double aimCoordinate(Wall w, double wallHalf, double half, double overdriveIn) {
        return (w.nx != 0 ? w.nx : w.ny) * (wallHalf - half + overdriveIn);
    }

    /**
     * @param ex,ey,eh        the robot's pose ESTIMATE after it stopped against the wall
     * @param reachTolIn      the estimate must already be within this of the expected contact position,
     *                        otherwise the robot is stuck on something else and no fix is made
     */
    public static Result compute(Wall w, double ex, double ey, double eh, double wallHalf,
                                 double halfLength, double halfWidth, double maxSkewDeg, double reachTolIn) {
        double half = faceHalf(w, eh, halfLength, halfWidth, maxSkewDeg);
        if (Double.isNaN(half)) return new Result(false, ex, ey, eh, "NOT SQUARE TO WALL");
        double sign = (w.nx != 0 ? w.nx : w.ny);
        double along = sign * (w.nx != 0 ? ex : ey);                // toward the wall is positive
        double expected = wallHalf - half;
        if (Math.abs(along - expected) > reachTolIn) {
            return new Result(false, ex, ey, eh, String.format("STOPPED %.0f IN FROM WALL", expected - along));
        }
        double r = rel(w, eh);
        long k = Math.round(r / 90.0);
        double heading = ((w.angleDeg - 90.0 * k) % 360 + 360) % 360;
        double nxPos = w.nx != 0 ? sign * expected : ex;
        double nyPos = w.ny != 0 ? sign * expected : ey;
        return new Result(true, nxPos, nyPos, heading, "OK");
    }
}
