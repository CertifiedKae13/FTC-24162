package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * Manual check of the flower vision, using the SAME validation as Flower Auton (LimelightVision.look per frame, then
 * LimelightVision.summarize over a LL_FLOWER_LOOK_S window, exactly like MainAuto2.lookForSteady). Point the robot
 * at a flower and read the verdict. The robot does not move.
 *
 * It also shows the slot-0 AprilTag reader side by side, because that reader is the ONLY source of
 * "NO VALID TARGET": if the flower line says VALID while the AprilTag line says NO VALID TARGET, the flower
 * script was never the cause.
 *
 * A = flower slot (LL_FLOWER_PIPELINE), B = AprilTag slot (LL_PIPELINE).
 */
@TeleOp(name = "Flower Vision Test", group = "Tools")
public class FlowerVisionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        LimelightVision ll = new LimelightVision(hardwareMap);
        int flowerSlot = (int) AutoConfig.LL_FLOWER_PIPELINE;
        int tagSlot = (int) AutoConfig.LL_PIPELINE;
        int slot = flowerSlot;
        ll.usePipeline(slot);

        ArrayList<LimelightVision.Sighting> got = new ArrayList<>();
        LimelightVision.Sighting last = null;
        LimelightVision.Sighting decision = null;
        double lastStamp = Double.NaN;
        int windows = 0, validWindows = 0;
        ElapsedTime window = new ElapsedTime();

        telemetry.addLine("Flower Vision Test: press START, point the robot at a flower (it does not move)");
        telemetry.addData("Limelight", ll.debugLine());
        telemetry.update();
        waitForStart();
        window.reset();

        while (opModeIsActive()) {
            // Slot selection (switching restarts the decision window)
            int want = gamepad1.a ? flowerSlot : gamepad1.b ? tagSlot : slot;
            if (want != slot) {
                slot = want;
                ll.usePipeline(slot);
                got.clear(); decision = null; windows = validWindows = 0; lastStamp = Double.NaN; window.reset();
            }

            // 1) Per-frame flower check, same call as lookForSteady; only NEW frames that saw a target count
            last = ll.look(slot);
            if (last.seen && last.frameStamp != lastStamp) {
                got.add(last);
                lastStamp = last.frameStamp;
            }
            // 2) Decision, same rule as lookForSteady: stop at LL_STEADY_FRAMES frames or after LL_FLOWER_LOOK_S
            if (got.size() >= (int) AutoConfig.LL_STEADY_FRAMES || window.seconds() >= AutoConfig.LL_FLOWER_LOOK_S) {
                decision = LimelightVision.summarize(got, last);
                windows++;
                if (decision.seen) validWindows++;
                got.clear();
                window.reset();
            }

            // 3) The AprilTag reader, which is where "NO VALID TARGET" comes from
            LimelightVision.Reading tags = ll.update();

            telemetry.addData("Slot selected", "%d (%s)   A = flower %d, B = AprilTag %d", slot,
                    slot == flowerSlot ? "flower" : slot == tagSlot ? "AprilTag" : "?", flowerSlot, tagSlot);
            telemetry.addData("Raw Limelight", ll.debugLine());
            telemetry.addData("Flower frame", last.seen
                    ? String.format("SEEN tx %.1f  py found %s  spacing %s", last.txDeg, fmt(last.pythonFound),
                    fmt(last.spacingPx))
                    : last.status);
            if (decision == null) {
                telemetry.addData("FLOWER VERDICT", "collecting...");
            } else if (decision.seen) {
                telemetry.addData("FLOWER VERDICT", "VALID  %s  tx %.1f  spacing %s", decision.status,
                        decision.txDeg, fmt(decision.spacingPx));
            } else {
                telemetry.addData("FLOWER VERDICT", "NOT VALID: %s", decision.status);
            }
            telemetry.addData("Valid windows", "%d of %d", validWindows, windows);
            telemetry.addData("AprilTag reader (slot " + tagSlot + ")", tags.valid ? "OK" : tags.status);
            telemetry.addLine(explain(slot, flowerSlot, decision, tags));
            telemetry.update();
            idle();
        }
        ll.usePipeline(tagSlot);
        ll.close();
    }

    /** One-line reading of what the screen means for the "NO VALID TARGET" question. */
    private static String explain(int slot, int flowerSlot, LimelightVision.Sighting decision,
                                  LimelightVision.Reading tags) {
        if (decision == null) return "";
        boolean noValid = !tags.valid && tags.status.startsWith("NO VALID TARGET");
        if (slot == flowerSlot && decision.seen) {
            return "=> Flower script works. 'NO VALID TARGET' is the AprilTag reader, not the flower.";
        }
        if (slot == flowerSlot) {
            return "=> Flower NOT found by the script: check the Limelight stream for a yellow box (see reason above).";
        }
        return noValid ? "=> On the AprilTag slot: 'NO VALID TARGET' = no AprilTag in view (expected at a flower)."
                : "=> On the AprilTag slot.";
    }

    private static String fmt(double v) { return Double.isNaN(v) ? "-" : String.format("%.1f", v); }
}
