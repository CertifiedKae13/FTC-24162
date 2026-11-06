package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Shows the shot solution live: the three estimates (hive tilt, odometry, AprilTag), how they were combined, and the
 * resulting distance / height / RPM / angle / turn. The robot never drives.
 *
 * INIT: set the start pose like the autos (d-pad L/R alliance, U/D y +-1 in, bumpers +-6 in), robot on its wall.
 * RUN: push the robot by hand to a shooting spot. HOLD right trigger = spin the flywheel + set the hood for the
 * current solution (released = flywheel stops). It never feeds a ball.
 */
@TeleOp(name = "Shot Solver Test", group = "Tools")
public class ShotSolverTest extends LinearOpMode {
    private static final double WALL_IN = 72.0;

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        Drivetrain2 drivetrain = new Drivetrain2(hardwareMap);
        drivetrain.stop();
        Odometry2 odometry = new Odometry2(hardwareMap, drivetrain);
        odometry.init();
        LimelightVision ll = new LimelightVision(hardwareMap);
        ll.usePipeline((int) AutoConfig.LL_PIPELINE);   // AprilTags
        Shooter shooter = new Shooter(hardwareMap);
        ElapsedTime clock = new ElapsedTime();

        boolean red = true;
        double startY = FieldStart.Y_IN;
        boolean up = false, down = false, left = false, right = false, lb = false, rb = false, dirty = true;
        while (!isStarted() && !isStopRequested()) {
            odometry.update();
            odometry.updateReset();
            if (gamepad1.dpad_left && !left) { red = true; dirty = true; }
            if (gamepad1.dpad_right && !right) { red = false; dirty = true; }
            if (gamepad1.dpad_up && !up) { startY += 1; dirty = true; }
            if (gamepad1.dpad_down && !down) { startY -= 1; dirty = true; }
            if (gamepad1.right_bumper && !rb) { startY += 6; dirty = true; }
            if (gamepad1.left_bumper && !lb) { startY -= 6; dirty = true; }
            left = gamepad1.dpad_left; right = gamepad1.dpad_right; up = gamepad1.dpad_up;
            down = gamepad1.dpad_down; lb = gamepad1.left_bumper; rb = gamepad1.right_bumper;
            startY = Range.clip(startY, -AutoConfig.START_Y_MAX_IN, AutoConfig.START_Y_MAX_IN);
            double startX = (red ? -1 : 1) * (WALL_IN - AutoConfig.ROBOT_HALF_LENGTH_IN);
            double startH = red ? 0.0 : 180.0;
            if (dirty) { odometry.setFieldPose(startX, startY, startH); dirty = false; }
            ll.setAlliance(red);
            telemetry.addData("Start pose", "%s  x %.0f  y %.0f  h %.0f", red ? "RED" : "BLUE", startX, startY, startH);
            telemetry.addLine("D-pad L/R = alliance, U/D = y +-1 in, bumpers = y +-6 in");
            telemetry.addData("Pinpoint", odometry.isReady() ? "READY" : "NOT READY (keep still)");
            telemetry.addData("Shooter hardware", shooter.hasHardware() ? "launchMotor + launchServo found" : "MISSING");
            telemetry.update();
        }

        while (opModeIsActive()) {
            odometry.update();
            drivetrain.stop();
            LimelightVision.Reading r = ll.update(odometry.getHeadingDeg());
            ShotSolver.Solution s = shooter.solve(red, odometry, r, clock.seconds());

            boolean fire = gamepad1.right_trigger > 0.5;
            boolean applied = fire && shooter.apply(s);
            if (!fire) shooter.stop();

            telemetry.addData("Pose (odometry)", "x %.1f  y %.1f  h %.0f  %s", odometry.getX(), odometry.getY(),
                    odometry.getHeadingDeg(), odometry.isReady() ? "" : "NOT READY");
            telemetry.addData("Limelight", r.valid ? String.format("%d tags, %s hive", r.tagCount,
                    r.tagCluster < 0 ? "no" : r.hiveIsRed ? "RED" : "BLUE") : r.status);
            telemetry.addData("1 Hive tilt", "%s deg (%s)", fmt(s.tiltDeg), s.tiltSource);
            telemetry.addData("Target cell", "%s end of %s hive", s.cellSide > 0 ? "+Y" : "-Y", red ? "RED" : "BLUE");
            telemetry.addData("2 Pose odometry", est(s.odo));
            telemetry.addData("3 Pose AprilTag", est(s.tag) + (r.valid && r.hasPose ? "  (" + r.poseSource + ")" : ""));
            telemetry.addData("POSE (combined)", s.fused == null || !s.fused.ok ? "-" : est(s.fused) + "  from " + s.used);
            if (!s.warnings.isEmpty()) telemetry.addData("Warnings", s.warnings);
            telemetry.addData("Shot", s.ok
                    ? String.format("dist %.1f  rise %.1f  turn %+.1f deg  rpm %.0f  angle %.1f deg (%s arc)",
                    s.distanceIn, s.heightIn, s.turnDeg, s.rpm, s.angleDeg, s.highArc ? "high" : "flat")
                    : "NO SHOT: " + s.reason);
            telemetry.addData("Flywheel", fire ? (applied ? String.format("%.0f / %.0f rpm %s", shooter.rpm(), s.rpm,
                    shooter.atSpeed() ? "AT SPEED" : "spinning up") : "not applied (no valid shot)")
                    : "off (hold right trigger)");
            telemetry.update();
        }
        shooter.stop();
        ll.close();
        drivetrain.stop();
    }

    private static String est(ShotSolver.Estimate e) {
        if (e == null) return "-";
        if (!e.ok) return "none: " + e.source;
        return String.format("x %.1f  y %.1f  (+-%.1f in)", e.x, e.y, e.sigma);
    }

    private static String fmt(double v) { return Double.isNaN(v) ? "-" : String.format("%.1f", v); }
}
