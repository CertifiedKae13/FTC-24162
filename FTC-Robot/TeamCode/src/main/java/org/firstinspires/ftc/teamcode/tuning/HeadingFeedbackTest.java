package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/** Isolates headingGain with one 90-degree turn and return. */
public final class HeadingFeedbackTest extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        if (!TuningSafety.calibrated()) {
            telemetry.addLine(TuningSafety.missing()); telemetry.update();
            waitForStart(); return;
        }
        telemetry.addLine("Set only headingGain above zero. Clear rotation space.");
        telemetry.addLine("Graph Dashboard target vs actual heading. One cycle per run.");
        telemetry.update();
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        waitForStart();
        if (!opModeIsActive()) return;
        try {
            Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                    .turn(Math.PI / 2).turn(-Math.PI / 2).build());
        } finally { drive.setDrivePowers(TuningSafety.ZERO_POWER); }
    }
}
