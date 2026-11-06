package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

/** Slide 12: slow driving while comparing Dashboard field pose to the robot. */
public final class LocalizationTest extends LinearOpMode {
    private static final double SPEED = 0.35;
    @Override public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        telemetry.addLine("Slow field-localization check. Driver Station sticks drive at 35%.");
        telemetry.addLine("Use Dashboard field overlay; STOP halts motors.");
        telemetry.update();
        waitForStart();
        try {
            while (opModeIsActive()) {
                drive.updatePoseEstimate();
                if (!drive.pinpointReady()) {
                    drive.setDrivePowers(TuningSafety.ZERO_POWER);
                    telemetry.addLine("Pinpoint not READY; drive stopped.");
                    telemetry.update();
                    sleep(20);
                    continue;
                }
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(-gamepad1.left_stick_y * SPEED,
                                -gamepad1.left_stick_x * SPEED),
                        -gamepad1.right_stick_x * SPEED));
                Pose2d pose = drive.localizer.getPose();
                telemetry.addData("X forward / Y left (in)", "%.2f / %.2f",
                        pose.position.x, pose.position.y);
                telemetry.addData("Heading CCW (deg)", "%.1f",
                        Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), pose);
                packet.put("xIn", pose.position.x);
                packet.put("yIn", pose.position.y);
                packet.put("headingDeg", Math.toDegrees(pose.heading.toDouble()));
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } finally { drive.setDrivePowers(TuningSafety.ZERO_POWER); }
    }
}
