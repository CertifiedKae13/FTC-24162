package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/** Slide 14: one bounded spline after all previous values are saved. */
public final class SplineTest extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        if (!TuningSafety.calibrated()) {
            telemetry.addLine(TuningSafety.missing()); telemetry.update();
            waitForStart(); return;
        }
        telemetry.addLine("Final spline: start (0,0) to (18,12), then (0,24) inches.");
        telemetry.addLine("Clear space and be ready to STOP.");
        telemetry.update();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();
        if (!opModeIsActive()) return;
        try {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                    .splineTo(new Vector2d(18, 12), Math.PI / 2)
                    .splineTo(new Vector2d(0, 24), Math.PI)
                    .build());
        } finally { drive.setDrivePowers(TuningSafety.ZERO_POWER); }
    }
}
