package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/** Slide 13: repeated forward/backward feedback tuning. */
public final class ManualFeedbackTuner extends LinearOpMode {
    // 24 inches keeps the initial test within a normal clear FTC field lane.
    public static double DISTANCE = 24;

    @Override public void runOpMode() throws InterruptedException {
        if (!TuningSafety.calibrated()) {
            telemetry.addLine("Finish push/ramp/offset tuning before feedback tests.");
            telemetry.addLine(TuningSafety.missing());
            telemetry.update();
            waitForStart();
            return;
        }
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        telemetry.addLine("24-inch forward/backward tuning; keep lane clear.");
        telemetry.addLine("Use Dashboard to change one gain at a time. STOP ends the test.");
        telemetry.update();
        waitForStart();
        try {
            while (opModeIsActive()) {
                Actions.runBlocking(drive.actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(DISTANCE).lineToX(0).build());
                if (drive.hasTuningFault()) break;
            }
        } finally {
            drive.setDrivePowers(TuningSafety.ZERO_POWER);
        }
    }
}
