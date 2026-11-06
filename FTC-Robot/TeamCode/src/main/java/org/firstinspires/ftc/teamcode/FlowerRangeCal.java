package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Measures LL_FLOWER_RANGE_K for flower localization (FlowerFix): K = depth (in) x post spacing (px).
 *
 * 1. Load the flower SnapScript in Limelight slot LL_FLOWER_PIPELINE.
 * 2. Put the robot square to a flower, crosshair on it (tx near 0).
 * 3. Measure along the floor from the Limelight LENS to the flower's posts (the line between their centres).
 * 4. D-pad up/down = distance +-1 in, bumpers +-6 in, until it matches the tape measure.
 * 5. Press A to record a sample. Repeat at 3-4 distances (e.g. 18, 24, 36, 48 in).
 * 6. Copy the mean K it shows into AutoConfig.LL_FLOWER_RANGE_K (Dashboard), then LL_FIX_APPLY = 1.
 * The robot does not move. B clears the samples.
 */
@TeleOp(name = "Flower Range Cal", group = "Tools")
public class FlowerRangeCal extends LinearOpMode {
    private static final int WINDOW = 15;   // median over this many recent pair frames

    @Override
    public void runOpMode() {
        LimelightVision ll = new LimelightVision(hardwareMap);
        int pipeline = (int) AutoConfig.LL_FLOWER_PIPELINE;
        ll.usePipeline(pipeline);
        double distanceIn = 24.0;
        ArrayList<Double> recent = new ArrayList<>();
        ArrayList<Double> ks = new ArrayList<>();
        double lastStamp = Double.NaN;
        boolean pu = false, pd = false, plb = false, prb = false, pa = false, pb = false;

        telemetry.addLine("Flower Range Cal: press START (robot does not move)");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            // Distance entry (rising edges only)
            if (gamepad1.dpad_up && !pu) distanceIn += 1;
            if (gamepad1.dpad_down && !pd) distanceIn -= 1;
            if (gamepad1.right_bumper && !prb) distanceIn += 6;
            if (gamepad1.left_bumper && !plb) distanceIn -= 6;
            distanceIn = Math.max(6, distanceIn);

            // Only new frames that saw the post PAIR count
            LimelightVision.Sighting s = ll.look(pipeline);
            if (s.seen && !Double.isNaN(s.spacingPx) && s.frameStamp != lastStamp) {
                recent.add(s.spacingPx);
                if (recent.size() > WINDOW) recent.remove(0);
                lastStamp = s.frameStamp;
            }
            double spacing = median(recent);

            if (gamepad1.a && !pa && !Double.isNaN(spacing) && recent.size() >= 5) {
                ks.add(distanceIn * spacing);
            }
            if (gamepad1.b && !pb) ks.clear();
            pu = gamepad1.dpad_up; pd = gamepad1.dpad_down; plb = gamepad1.left_bumper; prb = gamepad1.right_bumper;
            pa = gamepad1.a; pb = gamepad1.b;

            telemetry.addData("Limelight", s.seen ? s.status : "not seen (" + s.status + ")");
            telemetry.addData("tx", s.seen ? String.format("%.1f deg %s", s.txDeg,
                    Math.abs(s.txDeg) > 3 ? "(turn to centre it)" : "ok") : "-");
            telemetry.addData("Post spacing", Double.isNaN(spacing) ? "- (need both posts)"
                    : String.format("%.1f px (median of %d)", spacing, recent.size()));
            telemetry.addData("Tape distance (lens->posts)", "%.0f in   [dpad +-1, bumpers +-6]", distanceIn);
            telemetry.addData("Range with current K", Double.isNaN(spacing) ? "-"
                    : String.format("%.1f in (K %.0f)", AutoConfig.LL_FLOWER_RANGE_K / spacing,
                    AutoConfig.LL_FLOWER_RANGE_K));
            if (!ks.isEmpty()) {
                double mean = 0, min = Double.POSITIVE_INFINITY, max = 0;
                for (double k : ks) { mean += k; min = Math.min(min, k); max = Math.max(max, k); }
                mean /= ks.size();
                telemetry.addData("Samples", "%d   K min %.0f  max %.0f", ks.size(), min, max);
                telemetry.addData(">> SET LL_FLOWER_RANGE_K", "%.0f", mean);
                if (max - min > 0.08 * mean) telemetry.addLine("Spread > 8%: re-check distances / squareness");
            }
            telemetry.addLine("A = record sample   B = clear samples");
            telemetry.update();
            idle();
        }
        ll.usePipeline((int) AutoConfig.LL_PIPELINE);
        ll.close();
    }

    private static double median(ArrayList<Double> v) {
        if (v.isEmpty()) return Double.NaN;
        double[] s = new double[v.size()];
        for (int i = 0; i < s.length; i++) s[i] = v.get(i);
        Arrays.sort(s);
        int n = s.length;
        return n % 2 == 1 ? s[n / 2] : 0.5 * (s[n / 2 - 1] + s[n / 2]);
    }
}
