package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Flower Auton: visits each of the four FLOWERS (the green posts on the field walls) and stops directly in front of
 * each one, facing it, with FLOWER_GAP_IN between the robot's front and the flower's front face. Same robot setup, start-pose selector, braking and
 * safety stops as MainAuto2 (it reuses MainAuto2's drive / turn helpers and only replaces the routine).
 *
 * Path per flower: drive to a staging point STAGE_IN in front of the flower (turning to face each leg first), turn to
 * face the flower, drive straight in to the stop point, then back straight out to the staging point. Between flowers
 * it goes through a corner waypoint at (+-48, +-48) so the legs stay clear of the hive and of the other flowers.
 * Checked on paper against the hive footprint and flower positions: when stopped the robot centre is 15.6 in
 * from the flower centre (3.6 in flower front + 3 in gap + 9 in robot front); in transit it stays >= 14 in from the
 * hive's outline and farther from every flower.
 *
 * Flower positions: from FIRST's field CAD (FLOWER pipe-square centre 68.04 in from the field centre, 23.39 in off
 * the wall midpoint), placed in this project's frame like FieldMap.FLOWERS (red wall at -X, matching the BIOBUZZ
 * field image on the Dashboard). Check them on your field before running.
 */
@Autonomous(name = "Flower Auton", group = "Competition")
public class FlowerAuto extends MainAuto2 {
    // ===== TUNABLE =====
    /** Gap from the robot's front to the flower's FRONT FACE when stopped (inches). */
    static final double FLOWER_GAP_IN = 3.0;
    /**
     * Flower size: distance from the flower's position (its centre) to its front face, toward the field. The top
     * ring is 2.40 in from the opening centre to its front edge (Competition Manual Figure 9-12); the base plate on
     * the tiles sticks out ~1.2 in further (estimated from the same figure's side view), and a low robot meets the
     * base first. Measure your flower and correct these if needed.
     */
    static final double FLOWER_TOP_RING_FRONT_IN = 2.40;
    static final double FLOWER_BASE_OVERHANG_IN = 1.2;
    /** How exactly the stop point must be reached (inches). */
    static final double STOP_TOLERANCE_IN = 0.3;
    /** Robot centre to its front bumper (inches); 9 for an 18 in robot. */
    static final double ROBOT_FRONT_IN = 9.0;
    /** How far in front of the flower to line up and turn (flower centre to robot centre, inches). */
    static final double STAGE_IN = 26.0;
    static final double POWER = 0.6;

    /** {x, y, heading that faces the flower}, field frame, inches/degrees. Order: red wall, audience, blue wall, far. */
    static final double[][] FLOWERS = {
            {-68.04, -23.39, 180},
            {23.39, -68.04, 270},
            {68.04, 23.39, 0},
            {-23.39, 68.04, 90},
    };
    /** Corner waypoints used between consecutive flowers. */
    static final double[][] CORNERS = {{-48, -48}, {48, -48}, {48, 48}};

    @Override
    void runSequence() {
        drive(POWER, 24.0);                                   // leave the wall straight ahead before any turn
        for (int i = 0; i < FLOWERS.length && opModeIsActive() && !isAborted(); i++) {
            double[] f = FLOWERS[i];
            double hx = Math.cos(Math.toRadians(f[2])), hy = Math.sin(Math.toRadians(f[2]));
            double flowerFront = FLOWER_TOP_RING_FRONT_IN + FLOWER_BASE_OVERHANG_IN;
            double stopIn = flowerFront + FLOWER_GAP_IN + ROBOT_FRONT_IN;          // flower centre -> robot centre
            double sx = f[0] - hx * STAGE_IN, sy = f[1] - hy * STAGE_IN;     // staging point
            double px = f[0] - hx * stopIn, py = f[1] - hy * stopIn;         // stop point
            faceAndDrive(POWER, sx, sy);
            turnTo(f[2]);                                     // face the flower
            // The flower is only in the Limelight's view now that the robot faces it: check it is really there.
            LimelightVision.Sighting seen = lookForSteady((int) AutoConfig.LL_FLOWER_PIPELINE, AutoConfig.LL_FLOWER_LOOK_S);
            // Robot is stopped and facing the flower: use the same sighting as an absolute position fix.
            telemetry.addData("Flower fix " + (i + 1), flowerFix(seen, FLOWERS));
            if (seen.seen && Math.abs(seen.txDeg) > AutoConfig.LL_FLOWER_ALIGN_DEG) {
                turn(-seen.txDeg);                            // tx + = flower to the right -> turn right (CW, negative)
            }
            telemetry.addData("Limelight flower " + (i + 1), seen.seen
                    ? String.format("%s  tx %.1f  ty %.1f  area %.2f%%", seen.status, seen.txDeg, seen.tyDeg, seen.areaPct)
                    : "not detected (" + seen.status + ")");
            telemetry.update();
            drivePrecise(POWER, px, py, STOP_TOLERANCE_IN);   // straight in, still facing it: STOP here
            if (isAborted()) return;
            telemetry.addData("Flower", "%d of %d reached", i + 1, FLOWERS.length);
            telemetry.update();
            pauseUpdating(0.5);                               // show it is stopped (keeps the odometry fresh)
            drive(POWER, sx, sy);                             // straight back out (tail first)
            if (i < CORNERS.length) faceAndDrive(POWER, CORNERS[i][0], CORNERS[i][1]);
        }
    }
}
