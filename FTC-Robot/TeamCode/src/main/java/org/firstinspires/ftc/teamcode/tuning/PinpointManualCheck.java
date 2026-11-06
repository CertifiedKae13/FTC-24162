package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;

/** Read-only, hand-moved checks. These modes never command drive motor power. */
public abstract class PinpointManualCheck extends LinearOpMode {
    protected abstract String testName();
    protected abstract String instruction();
    protected abstract double measurement(double dxIn, double dyIn, double dHeadingDeg);
    protected abstract String unit();

    @Override public final void runOpMode() throws InterruptedException {
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap,
                MecanumDrive.PARAMS.inPerTick, new Pose2d(0, 0, 0));
        GoBildaPinpointDriver pinpoint = localizer.driver;
        telemetry.addLine("Keep robot stationary during INIT while Pinpoint calibrates.");
        telemetry.addLine("START begins. A resets the displayed baseline.");
        telemetry.addLine(instruction());
        telemetry.update();
        waitForStart();
        boolean baselineSet = false;
        boolean previousA = false;
        double startX = 0, startY = 0, startHeading = 0;
        while (opModeIsActive()) {
            pinpoint.update();
            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            if (status == GoBildaPinpointDriver.DeviceStatus.READY) {
                double x = pinpoint.getPosX(DistanceUnit.INCH);
                double y = pinpoint.getPosY(DistanceUnit.INCH);
                double heading = pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);
                if (Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(heading)) {
                    if (!baselineSet || (gamepad1.a && !previousA)) {
                        startX = x; startY = y; startHeading = heading;
                        baselineSet = true;
                    }
                    double dx = x - startX, dy = y - startY, dh = heading - startHeading;
                    telemetry.addData(testName(), "%.2f %s", measurement(dx, dy, dh), unit());
                    telemetry.addData("X forward / Y left (in)", "%.2f / %.2f", dx, dy);
                    telemetry.addData("Heading CCW (deg)", "%.1f", dh);
                    telemetry.addData("Raw parallel / perpendicular ticks", "%d / %d",
                            pinpoint.getEncoderX(), pinpoint.getEncoderY());
                } else telemetry.addLine("Non-finite Pinpoint reading: check sensor and wiring.");
            } else {
                baselineSet = false;
                telemetry.addData("Pinpoint", "Not ready: %s", status);
            }
            previousA = gamepad1.a;
            telemetry.addLine(instruction());
            telemetry.addLine("A: reset displayed baseline; STOP: finish");
            telemetry.update();
            sleep(20);
        }
    }
}
