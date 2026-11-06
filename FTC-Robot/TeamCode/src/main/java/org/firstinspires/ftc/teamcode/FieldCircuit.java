package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * FieldCircuit: drives a rectangular loop around the field boundary to test odometry/navigation.
 *
 * SETUP: place the robot at FieldStart's pose (default: field center, facing +X) BEFORE init and
 * keep it still while Pinpoint calibrates. Field frame: origin at the field center, heading 0 =
 * facing +X, CCW-positive; the field is 144 in x 144 in, so the walls are at +/-72 in.
 *
 * ROUTE: center -> corner C1 (+X,-Y) -> C2 (+X,+Y) -> C3 (-X,+Y) -> C4 (-X,-Y) -> C1 -> center,
 * facing along each side. Corners are inset from the walls by CLEARANCE_IN (default 18 in, for a
 * robot about 18 in wide). Change CLEARANCE_IN to move the loop closer to or farther from the walls.
 *
 * GOOD RESULT: it finishes at the start with a small position error (a few inches or less) and
 * heading error (a few degrees); the final summary shows both. Large drift = odometry offsets /
 * pod directions / wheel slip need attention.
 *
 * SAFETY: SPEED caps power for this OpMode only (start low). DRY_RUN only prints the waypoints.
 * STOP aborts immediately. Any failed leg aborts the rest of the run.
 */
@Autonomous(name = "FieldCircuit", group = "Test")
public class FieldCircuit extends LinearOpMode {
    private static final double SPEED = 0.4;        // 0.1 to 1.0; navigation power cap for this test
    private static final boolean DRY_RUN = false;   // true = only list the planned waypoints
    private static final double FIELD_HALF_IN = 72.0;
    private static final double CLEARANCE_IN = 18.0;

    private static final double EDGE_IN = FIELD_HALF_IN - CLEARANCE_IN; // corner coordinate
    // Corners, counter-clockwise starting at (+X,-Y). Heading along each side in degrees.
    private static final double[][] LEGS = {
            // x,        y,        heading while driving this leg
            { EDGE_IN, -EDGE_IN,  0.0 },    // C1: from the center
            { EDGE_IN,  EDGE_IN, 90.0 },    // C2: up the +X side
            {-EDGE_IN,  EDGE_IN, 180.0 },   // C3: along the +Y side
            {-EDGE_IN, -EDGE_IN, 270.0 },   // C4: down the -X side
            { EDGE_IN, -EDGE_IN,  0.0 },    // C1: along the -Y side
    };

    @Override
    public void runOpMode() {
        NavActions nav = new NavActions(this);
        nav.setSpeed(SPEED);
        nav.init();

        telemetry.addLine("FieldCircuit " + (DRY_RUN ? "(DRY RUN) " : "") + "ready. Speed " + SPEED);
        telemetry.addData("Start", "x %.1f y %.1f h %.1f", nav.getX(), nav.getY(), nav.getHeading());
        for (int i = 0; i < LEGS.length; i++) {
            telemetry.addData("Leg " + (i + 1), "(%.0f, %.0f) face %.0f", LEGS[i][0], LEGS[i][1], LEGS[i][2]);
        }
        telemetry.addData("Then", "return to (0, 0) facing 0");
        telemetry.update();
        waitForStart();
        if (!opModeIsActive() || DRY_RUN) return;

        double startX = nav.getX(), startY = nav.getY(), startH = nav.getHeading();
        boolean ok = true;
        for (int i = 0; ok && i < LEGS.length; i++) {
            nav.setLabel("Leg " + (i + 1) + " -> (" + LEGS[i][0] + ", " + LEGS[i][1] + ")");
            // Face along the side first, then travel; heading is kept while driving.
            ok = nav.turnTo(LEGS[i][2]) && nav.driveTo(LEGS[i][0], LEGS[i][1], LEGS[i][2]);
            telemetry.log().add("Leg %d %s  pose (%.1f, %.1f, %.1f)", i + 1, ok ? "OK" : "FAILED",
                    nav.getX(), nav.getY(), nav.getHeading());
        }
        if (ok) {
            nav.setLabel("Return to start");
            ok = nav.driveTo(startX, startY, startH);
        }
        nav.stop();

        double posErr = Math.hypot(nav.getX() - startX, nav.getY() - startY);
        double headErr = Math.abs(((nav.getHeading() - startH + 540.0) % 360.0) - 180.0);
        telemetry.addData("RESULT", ok ? "COMPLETED" : "ABORTED: " + nav.getLastResult());
        telemetry.addData("End error", "%.2f in position, %.2f deg heading", posErr, headErr);
        telemetry.update();
        while (opModeIsActive()) sleep(50);
    }
}
