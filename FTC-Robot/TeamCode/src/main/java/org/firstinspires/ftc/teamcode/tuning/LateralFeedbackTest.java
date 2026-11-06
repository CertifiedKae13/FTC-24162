package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/** Isolates lateralGain with one 24-inch left/right path. */
public final class LateralFeedbackTest extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        if (!TuningSafety.calibrated()) {
            telemetry.addLine(TuningSafety.missing()); telemetry.update();
            waitForStart(); return;
        }
        telemetry.addLine("Set only lateralGain above zero. Clear 24 inches left.");
        telemetry.addLine("Graph Dashboard target vs actual pose. One cycle per run.");
        telemetry.update();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();
        if (!opModeIsActive()) return;
        try {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                    .lineToY(24).lineToY(0).build());
        } finally { drive.setDrivePowers(TuningSafety.ZERO_POWER); }
    }
}
