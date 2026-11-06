package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Shows where the robot THINKS it is (its own pose estimate, never ground truth) and where it is
 * going. Two views, both in FieldMap's frame (inches, origin = field center, +X toward blue):
 *  1. Driver Station telemetry: a text minimap, E = estimate, T = target, G = final goal,
 *     '.' = the line from E to T, '#' = hive keep-out.
 *  2. FTC Dashboard (browser, http://192.168.43.1:8080/dash): the field with a line from the
 *     estimate to the target. Dashboard draws +X up / +Y left, so points are rotated into that frame.
 */
public final class PoseDisplay {
    private PoseDisplay() {}

    private static final int COLS = 24;              // 144 in / 24 = 6 in per column
    private static final int ROWS = 12;              // 144 in / 12 = 12 in per row (text is ~2x taller than wide)
    private static final double LINE_STEP_IN = 3.0;  // sampling step when rasterizing the E-T line

    private static boolean formatSet;

    /** Call every loop. goalX/goalY may be NaN. Heading in degrees. */
    public static void show(Telemetry t, double ex, double ey, double eh,
                            double tx, double ty, double goalX, double goalY) {
        if (!formatSet) {
            t.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
            formatSet = true;
        }
        char[][] g = new char[ROWS][COLS];
        for (char[] row : g) java.util.Arrays.fill(row, ' ');
        for (int r = 0; r < ROWS; r++) {
            for (int c = 0; c < COLS; c++) {
                double x = cellX(c), y = cellY(r);
                if (inFrame(x, y)) g[r][c] = '#';
            }
        }
        double dist = Math.hypot(tx - ex, ty - ey);
        int steps = (int) Math.ceil(dist / LINE_STEP_IN);
        for (int i = 1; i < steps; i++) {
            double f = (double) i / steps;
            put(g, ex + (tx - ex) * f, ey + (ty - ey) * f, '.');
        }
        if (!Double.isNaN(goalX)) put(g, goalX, goalY, 'G');
        put(g, tx, ty, 'T');
        put(g, ex, ey, 'E');   // drawn last so the estimate is never hidden

        String estLine = String.format("Est (robot thinks): x %.1f  y %.1f  h %.0f", ex, ey, eh);
        String tgtLine = String.format("Target: x %.1f  y %.1f  dist %.1f in", tx, ty, dist);
        String legend = "+Y up, +X right.  E=estimate T=target G=goal #=hive";
        t.addData("Est (robot thinks)", "x %.1f  y %.1f  h %.0f", ex, ey, eh);
        t.addData("Target", "x %.1f  y %.1f  dist %.1f in", tx, ty, dist);
        t.addLine(legend);
        t.addLine(border());
        for (int r = 0; r < ROWS; r++) t.addLine("|" + new String(g[r]) + "|");
        t.addLine(border());

        java.util.List<String> lines = new java.util.ArrayList<>();
        lines.add(estLine);
        lines.add(tgtLine);
        lines.add(legend);
        lines.add(border());
        for (int r = 0; r < ROWS; r++) lines.add("|" + new String(g[r]) + "|");
        lines.add(border());
        sendDashboard(ex, ey, eh, tx, ty, goalX, goalY);
    }

    // ---------------- Extras published with every Dashboard packet (graphs + gamepad view + battery) ----------------
    private static double extraBatteryV = Double.NaN;
    private static final double[] extraPad = new double[6];    // lx, ly, rx, ry, lt, rt
    private static final boolean[] extraButtons = new boolean[8]; // a b x y up down left right
    private static boolean extraPadSet;
    private static long lastPacketNs;
    private static LimelightVision.Reading extraVision;

    /** Latest Limelight reading, published with the next Dashboard packet (graph keys ll_*). Null clears it. */
    public static void setVision(LimelightVision.Reading r) { extraVision = r; }

    private static double extraTargetDistance = Double.NaN;

    /** Latest no-AprilTag target distance (inches) from LimelightVision, or NaN. */
    public static void setTargetDistance(double inches) { extraTargetDistance = inches; }

    /**
     * Latest battery voltage and gamepad 1 state. They are sent with the next Dashboard packet as numeric
     * telemetry (open the Dashboard Graph view and pick them), so you can watch the sticks, buttons and battery.
     */
    public static void setExtras(double batteryV, com.qualcomm.robotcore.hardware.Gamepad g) {
        extraBatteryV = batteryV;
        if (g == null) return;
        extraPad[0] = g.left_stick_x; extraPad[1] = g.left_stick_y;
        extraPad[2] = g.right_stick_x; extraPad[3] = g.right_stick_y;
        extraPad[4] = g.left_trigger; extraPad[5] = g.right_trigger;
        extraButtons[0] = g.a; extraButtons[1] = g.b; extraButtons[2] = g.x; extraButtons[3] = g.y;
        extraButtons[4] = g.dpad_up; extraButtons[5] = g.dpad_down; extraButtons[6] = g.dpad_left; extraButtons[7] = g.dpad_right;
        extraPadSet = true;
    }

    /** Numeric telemetry for the Dashboard Graph view: pose, target, distance, battery, gamepad. */
    private static void putTelemetry(TelemetryPacket p, double ex, double ey, double eh,
                                     double tx, double ty, double goalX, double goalY) {
        long nowNs = System.nanoTime();
        if (lastPacketNs != 0) p.put("loop_ms", (nowNs - lastPacketNs) / 1e6);   // update period, a latency/health check
        lastPacketNs = nowNs;
        p.put("est_x_in", ex);
        p.put("est_y_in", ey);
        p.put("est_heading_deg", eh);
        p.put("target_x_in", tx);
        p.put("target_y_in", ty);
        p.put("distance_to_target_in", Math.hypot(tx - ex, ty - ey));
        if (!Double.isNaN(goalX)) {
            p.put("goal_x_in", goalX);
            p.put("goal_y_in", goalY);
            p.put("distance_to_goal_in", Math.hypot(goalX - ex, goalY - ey));
        }
        if (!Double.isNaN(extraBatteryV)) p.put("battery_v", extraBatteryV);
        if (extraTargetDistance == extraTargetDistance) p.put("ll_target_distance_in", extraTargetDistance);  // not NaN
        if (extraVision != null) {
            p.put("ll_valid", extraVision.valid ? 1 : 0);
            p.addLine("Limelight: " + extraVision.status);
            if (extraVision.valid) {                      // only real values are sent, never zeros for "nothing seen"
                p.put("ll_tag_count", extraVision.tagCount);
                if (extraVision.hasHiveTilt) {
                    p.put("ll_hive_tilt_deg", extraVision.hiveTiltDeg);
                    p.put("ll_raw_pitch_deg", extraVision.rawPitch);   // for picking LL_TILT_AXIS
                    p.put("ll_raw_yaw_deg", extraVision.rawYaw);
                    p.put("ll_raw_roll_deg", extraVision.rawRoll);
                }
                if (extraVision.hasPose) {
                    p.put("ll_x_in", extraVision.x);
                    p.put("ll_y_in", extraVision.y);
                    p.put("ll_heading_deg", extraVision.headingDeg);
                }
            }
        }
        if (extraPadSet) {
            String[] axes = {"gp1_left_x", "gp1_left_y", "gp1_right_x", "gp1_right_y", "gp1_left_trigger", "gp1_right_trigger"};
            for (int i = 0; i < axes.length; i++) p.put(axes[i], extraPad[i]);
            String[] btn = {"gp1_a", "gp1_b", "gp1_x", "gp1_y", "gp1_dpad_up", "gp1_dpad_down", "gp1_dpad_left", "gp1_dpad_right"};
            for (int i = 0; i < btn.length; i++) p.put(btn[i], extraButtons[i] ? 1 : 0);
        }
    }

    // ---------------- FTC Dashboard (http://192.168.43.1:8080/dash) ----------------
    // Dashboard draws +X up / +Y left, so our point (x, y) is drawn at Dashboard (y, -x).
    private static void sendDashboard(double ex, double ey, double eh,
                                      double tx, double ty, double goalX, double goalY) {
        FtcDashboard dash = FtcDashboard.getInstance();
        if (dash == null) return;
        TelemetryPacket p = new TelemetryPacket();
        Canvas c = p.fieldOverlay();
        c.setAlpha(1.0);
        // Dashboard's built-in field picture is a DECODE (2025-26) one. Paint an opaque floor over it (this is
        // also the fallback if the image below fails to load), then draw the BIOBUZZ field image on top.
        c.setFill("#2b2b2b");
        fillPoly(c, -FieldMap.HALF - 2, -FieldMap.HALF - 2, FieldMap.FIELD_IN + 4, FieldMap.FIELD_IN + 4);
        // assets/dash/biobuzz.png is served by the Dashboard itself at /dash/biobuzz.png. The image has blue at the
        // top; rotating by PI about its centre puts our +X (blue) wall where this display draws +X.
        c.drawImage(AutoConfig.DARK_MAP ? "/dash/biobuzz-dark.png" : "/dash/biobuzz.png", 0, 0, FieldMap.FIELD_IN, FieldMap.FIELD_IN, Math.PI,
                FieldMap.HALF, FieldMap.HALF, false);
        c.setStrokeWidth(1);
        c.setStroke("#888888");
        poly(c, -FieldMap.HALF, -FieldMap.HALF, FieldMap.FIELD_IN, FieldMap.FIELD_IN);
        // Credit required by the FTControl License for the image (see assets/dash/biobuzz-credit.txt)
        c.setFill("#555555");
        c.fillText("Field image: Team Juice 16236 via FTControl Panels (bylazar.com)", -69, 70, "3px sans-serif", Math.PI / 2);
        if (AutoConfig.SHOW_GAMEPAD && extraPadSet) drawGamepad(c);
        // estimate -> target line, then the points: filled green = estimate, double red ring = target
        c.setStroke("#ffff00");
        c.setStrokeWidth(2);
        line(c, ex, ey, tx, ty);
        double h = Math.toRadians(eh);
        line(c, ex, ey, ex + 8 * Math.cos(h), ey + 8 * Math.sin(h));
        c.setFill("#00ff00");
        c.fillCircle(ey, -ex, 3);
        c.setStroke("#ff0000");
        c.strokeCircle(ty, -tx, 4);
        c.strokeCircle(ty, -tx, 2);
        if (!Double.isNaN(goalX)) {
            c.setStroke("#ff00ff");
            c.strokeCircle(goalY, -goalX, 5);
        }
        putTelemetry(p, ex, ey, eh, tx, ty, goalX, goalY);
        dash.sendTelemetryPacket(p);
    }

    /**
     * On-screen controller in the bottom-right of the field view: two sticks (dot = stick position), triggers as bars,
     * d-pad and A/B/X/Y lit when pressed. Drawn in our frame (X right, Y up on screen) via the same conversions.
     */
    private static void drawGamepad(Canvas c) {
        double cx = 46, cy = -60;                       // widget centre, our field inches (free corner of the map)
        c.setAlpha(0.9);
        c.setStrokeWidth(1);
        c.setStroke("#444444");
        ring(c, cx - 14, cy, 5);                        // left stick
        ring(c, cx + 2, cy, 5);                         // right stick
        c.setFill("#00c0ff");
        dot(c, cx - 14 + extraPad[0] * 4, cy - extraPad[1] * 4, 1.4);   // FTC stick y is up = negative
        dot(c, cx + 2 + extraPad[2] * 4, cy - extraPad[3] * 4, 1.4);
        c.setStroke("#ff8800");
        c.setStrokeWidth(2);
        line(c, cx - 19, cy + 8, cx - 19 + extraPad[4] * 10, cy + 8);   // left trigger bar
        line(c, cx - 1, cy + 8, cx - 1 + extraPad[5] * 10, cy + 8);     // right trigger bar
        c.setStrokeWidth(1);
        String[] colors = {"#22cc22", "#dd2222", "#2266ff", "#ddcc00"};   // A B X Y
        double[][] face = {{0, -3}, {3, 0}, {-3, 0}, {0, 3}};
        for (int i = 0; i < 4; i++) button(c, cx + 14 + face[i][0], cy + face[i][1], 1.3, extraButtons[i], colors[i]);
        double[][] dpad = {{0, 2.6}, {0, -2.6}, {-2.6, 0}, {2.6, 0}};     // up down left right
        for (int i = 0; i < 4; i++) button(c, cx - 14 + dpad[i][0], cy - 10 + dpad[i][1], 1.0, extraButtons[4 + i], "#ffffff");
        c.setAlpha(1.0);
    }

    private static void ring(Canvas c, double x, double y, double r) { c.strokeCircle(y, -x, r); }

    private static void dot(Canvas c, double x, double y, double r) { c.fillCircle(y, -x, r); }

    private static void button(Canvas c, double x, double y, double r, boolean pressed, String color) {
        if (pressed) { c.setFill(color); c.fillCircle(y, -x, r); }
        else { c.setStroke("#666666"); c.strokeCircle(y, -x, r); }
    }

    private static void line(Canvas c, double x1, double y1, double x2, double y2) {
        c.strokeLine(y1, -x1, y2, -x2);
    }

    private static void poly(Canvas c, double x, double y, double w, double h) {
        c.strokePolygon(new double[]{y, y + h, y + h, y}, new double[]{-x, -x, -(x + w), -(x + w)});
    }

    private static void fillPoly(Canvas c, double[] r) { fillPoly(c, r[0], r[1], r[2], r[3]); }

    private static void fillPoly(Canvas c, double x, double y, double w, double h) {
        c.fillPolygon(new double[]{y, y + h, y + h, y}, new double[]{-x, -x, -(x + w), -(x + w)});
    }

    private static String border() {
        StringBuilder b = new StringBuilder("+");
        for (int i = 0; i < COLS; i++) b.append('-');
        return b.append('+').toString();
    }

    private static double cellX(int col) { return -FieldMap.HALF + (col + 0.5) * (FieldMap.FIELD_IN / COLS); }
    private static double cellY(int row) { return FieldMap.HALF - (row + 0.5) * (FieldMap.FIELD_IN / ROWS); }

    private static boolean inFrame(double x, double y) {
        double[] f = FieldMap.HIVE_FRAME;
        return x >= f[0] && x <= f[0] + f[2] && y >= f[1] && y <= f[1] + f[3];
    }

    private static void put(char[][] g, double x, double y, char ch) {
        int col = (int) Math.floor((x + FieldMap.HALF) / (FieldMap.FIELD_IN / COLS));
        int row = (int) Math.floor((FieldMap.HALF - y) / (FieldMap.FIELD_IN / ROWS));
        col = Math.max(0, Math.min(COLS - 1, col));    // clamp: an off-field estimate shows on the edge
        row = Math.max(0, Math.min(ROWS - 1, row));
        g[row][col] = ch;
    }
}
