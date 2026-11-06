package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.canvas.Canvas;

/**
 * 2D map of the BIOBUZZ field in the same frame as FieldStart/Odometry2: inches, origin at the field
 * center, +X toward the BLUE alliance wall, +Y toward the top of Manual Figure 9-2, heading 0 = +X,
 * CCW positive. The red ALLIANCE AREA is at -X. Walls are at +/-72 in.
 *
 * Source of numbers: Competition Manual TU05 sec 9 and Event Field Setup Guide (see memory notes).
 * Hive and flower positions are within about +/-2 in (measured from the manual figures); confirm on
 * your physical field. Rows are {x, y, w, h} rectangles (x,y = lower-left corner) or {x, y, r} circles.
 */
public final class FieldMap {
    private FieldMap() {}

    public static final double FIELD_IN = 144.0;  // 12 ft x 12 ft, in inches: the ONE place the size is set
    public static final double HALF = FIELD_IN / 2;
    public static final double TILE = 24.0;       // 6 x 6 tiles, columns A-F (x -> +), rows 1-6 (y -> +)

    /** HIVE frame footprint: 49.46 in (X) by ~44 in (Y, along the hives), centered. */
    public static final double[] HIVE_FRAME = {-24.7, -22.0, 49.4, 44.0};
    /** Red HIVE at -X, blue at +X, 25.5 in center to center, each ~43 in long (Y) and ~12 in wide (X). */
    public static final double[][] HIVES = {{-12.75 - 6.0, -21.5, 12.0, 43.0}, {12.75 - 6.0, -21.5, 12.0, 43.0}};
    /** FLOWERS on each wall, ~24 in off the wall midpoint, {x, y, radius}. */
    public static final double[][] FLOWERS = {{-24, HALF, 5}, {HALF, 23, 5}, {23, -HALF, 5}, {-HALF, -23, 5}};
    /** LOADING ZONES 23 x 11 in against the wall: red tile A5, blue tile F2. */
    public static final double[][] LOADING = {{-HALF, 24, 11, 23}, {HALF - 11, -47, 11, 23}};
    /** GARDENS 23 x 2 in in opposite corners: red tile A1, blue tile F6. */
    public static final double[][] GARDENS = {{-HALF, -HALF, 23, 2}, {HALF - 23, HALF - 2, 23, 2}};
    /** ALLIANCE AREAS outside the field, 54 in deep, 97 in wide (y -48.5..48.5). */
    public static final double[][] ALLIANCE_AREAS = {{-HALF - 54, -48.5, 54, 97}, {HALF, -48.5, 54, 97}};

    /** Circles {x, y, r} covering the HIVE frame, used by MainAuto2 to route around it. */
    public static final double[][] KEEP_OUT = {
            {-12.0, -10.0, 13.0}, {-12.0, 10.0, 13.0},
            {  0.0, -10.0, 13.0}, {  0.0, 10.0, 13.0},
            { 12.0, -10.0, 13.0}, { 12.0, 10.0, 13.0},
    };

    /** Draw the field on an FTC Dashboard canvas (call before drawRobot). */
    public static void draw(Canvas c) {
        c.setStrokeWidth(1);
        for (int i = 0; i < 2; i++) {
            c.setStroke(i == 0 ? "#ff6b6b" : "#6b8bff");
            rect(c, ALLIANCE_AREAS[i]);
        }
        c.setStroke("#888888");
        c.strokeRect(-HALF, -HALF, 2 * HALF, 2 * HALF);
        for (int k = -2; k <= 2; k++) {
            c.strokeLine(k * TILE, -HALF, k * TILE, HALF);
            c.strokeLine(-HALF, k * TILE, HALF, k * TILE);
        }
        c.setStroke("#ff3030"); rect(c, LOADING[0]); rect(c, GARDENS[0]);
        c.setStroke("#3050ff"); rect(c, LOADING[1]); rect(c, GARDENS[1]);
        c.setStroke("#ffa500");
        for (double[] f : FLOWERS) c.strokeCircle(f[0], f[1], f[2]);
        c.setStroke("#cccccc"); rect(c, HIVE_FRAME);
        c.setStroke("#ff3030"); rect(c, HIVES[0]);
        c.setStroke("#3050ff"); rect(c, HIVES[1]);
        c.setStroke("#00aa00");
        for (double[] z : KEEP_OUT) c.strokeCircle(z[0], z[1], z[2]);
    }

    private static void rect(Canvas c, double[] r) { c.strokeRect(r[0], r[1], r[2], r[3]); }
}
