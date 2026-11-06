package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/** Isolates axialGain with one 24-inch forward/backward path. */
public final class AxialFeedbackTest extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        if (!TuningSafety.calibrated()) {
            telemetry.addLine(TuningSafety.missing()); telemetry.update();
            waitForStart(); return;
        }
        telemetry.addLine("Set only axialGain above zero. Clear 24 inches ahead.");
        telemetry.addLine("Graph Dashboard target vs actual pose. One cycle per run.");
        telemetry.update();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();
        if (!opModeIsActive()) return;
        try {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                    .lineToX(24).lineToX(0).build());
        } finally { drive.setDrivePowers(TuningSafety.ZERO_POWER); }
    }
}
