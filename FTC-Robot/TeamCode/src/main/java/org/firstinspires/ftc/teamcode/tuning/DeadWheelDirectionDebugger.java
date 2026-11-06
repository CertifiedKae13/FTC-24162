package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;

/** Read-only Pinpoint X/Y direction and I2C diagnostic; never powers motors. */
public final class DeadWheelDirectionDebugger extends LinearOpMode {
    private static final long LOG_INTERVAL_MS = 3000;
    private static final double DRIFT_TEST_SECONDS = 10;
    private static final int DRIFT_MAX_TICKS = 25; // about 0.05 in on a goBILDA pod


    @Override public void runOpMode() throws InterruptedException {
        GoBildaPinpointDriver pinpoint =
                hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        // Use the same pod type and directions as the robot.
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(PinpointLocalizer.PAR_DIRECTION,
                PinpointLocalizer.PERP_DIRECTION);
        telemetry.addLine("Keep the robot STILL. Recalibrating Pinpoint IMU...");
        telemetry.update();
        try {
            pinpoint.resetPosAndIMU();
        } catch (RuntimeException error) {
            telemetry.addData("Reset failed", error.getClass().getSimpleName());
        }
        // Show live status during INIT so a stuck Pinpoint is visible before START.
        boolean previousB = false;
        while (!isStarted() && !isStopRequested()) {
            try {
                pinpoint.update();
                telemetry.addData("Pinpoint status (INIT)", pinpoint.getDeviceStatus());
            } catch (RuntimeException error) {
                telemetry.addData("Pinpoint read error", error.getClass().getSimpleName());
            }
            telemetry.addLine("Wait for READY, then press START. B: recalibrate (keep still).");
            telemetry.addLine("Push forward: configured X position should increase.");
            telemetry.addLine("Press A to zero, then push left: configured Y should increase.");
            telemetry.addLine("This test never powers motors.");
            telemetry.update();
            if (gamepad1.b && !previousB) {
                try { pinpoint.resetPosAndIMU(); } catch (RuntimeException ignored) { }
            }
            previousB = gamepad1.b;
            sleep(100);
        }
        waitForStart();

        boolean baselineSet = false, previousA = false;
        double baselineX = 0, baselineY = 0;
        int baseTicksX = 0, baseTicksY = 0;
        int goodReads = 0, badReads = 0, consecutiveBadReads = 0;
        String lastReadError = "none";
        // Y-button stationary drift test: robot must not move for DRIFT_TEST_SECONDS.
        boolean driftTesting = false, previousY = false;
        long driftStartMs = 0;
        int driftStartX = 0, driftStartY = 0;
        String driftResult = "not run (press Y with robot still)";
        long lastLogMs = 0;
        String lastStatusText = "?", lastPoseText = "no READY data yet";
        while (opModeIsActive()) {
            try {
                pinpoint.update();
                GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
                telemetry.addData("Pinpoint status", status);
                lastStatusText = String.valueOf(status);
                if (status == GoBildaPinpointDriver.DeviceStatus.READY) {
                    double x = pinpoint.getPosX(DistanceUnit.INCH);
                    double y = pinpoint.getPosY(DistanceUnit.INCH);
                    if (Double.isFinite(x) && Double.isFinite(y)) {
                        goodReads++;
                        consecutiveBadReads = 0;
                        if (!baselineSet || (gamepad1.a && !previousA)) {
                            baselineX = x;
                            baselineY = y;
                            baseTicksX = pinpoint.getEncoderX();
                            baseTicksY = pinpoint.getEncoderY();
                            baselineSet = true;
                        }
                        telemetry.addData("Configured X delta (forward +)", "%.2f in", x - baselineX);
                        telemetry.addData("Configured Y delta (left +)", "%.2f in", y - baselineY);
                        telemetry.addData("Raw tick DELTA: X pod / Y pod", "%d / %d",
                                pinpoint.getEncoderX() - baseTicksX,
                                pinpoint.getEncoderY() - baseTicksY);
                        int rawX = pinpoint.getEncoderX(), rawY = pinpoint.getEncoderY();
                        if (gamepad1.y && !previousY && !driftTesting) {
                            driftTesting = true;
                            driftStartMs = System.currentTimeMillis();
                            driftStartX = rawX;
                            driftStartY = rawY;
                        }
                        if (driftTesting) {
                            double elapsed = (System.currentTimeMillis() - driftStartMs) / 1000.0;
                            int dX = rawX - driftStartX, dY = rawY - driftStartY;
                            driftResult = String.format("RUNNING %.0f/%.0fs: X %d, Y %d ticks",
                                    elapsed, DRIFT_TEST_SECONDS, dX, dY);
                            if (elapsed >= DRIFT_TEST_SECONDS) {
                                driftTesting = false;
                                boolean pass = Math.abs(dX) <= DRIFT_MAX_TICKS
                                        && Math.abs(dY) <= DRIFT_MAX_TICKS;
                                driftResult = String.format("%s: X pod %d, Y pod %d ticks in %.0fs"
                                                + " (limit +/-%d)%s", pass ? "PASS" : "FAIL",
                                        dX, dY, DRIFT_TEST_SECONDS, DRIFT_MAX_TICKS,
                                        pass ? "" : " -> pod/cable/board fault, robot not moving");
                            }
                        }
                        lastPoseText = String.format(
                                "Xdelta=%.2fin Ydelta=%.2fin tickDeltaX=%d tickDeltaY=%d rawX=%d rawY=%d heading=%.1f",
                                x - baselineX, y - baselineY, rawX - baseTicksX, rawY - baseTicksY,
                                rawX, rawY, pinpoint.getHeading(
                                        org.firstinspires.ftc.robotcore.external.navigation
                                                .AngleUnit.DEGREES));
                        telemetry.addData("Drift test (Y, robot STILL)", driftResult);
                        telemetry.addData("Heading (deg, should stay ~0 if pushed straight)", "%.1f",
                                pinpoint.getHeading(
                                        org.firstinspires.ftc.robotcore.external.navigation
                                                .AngleUnit.DEGREES));
                        telemetry.addData("Raw X / Y ticks (unreversed)", "%d / %d",
                                pinpoint.getEncoderX(), pinpoint.getEncoderY());
                    } else {
                        lastReadError = "non-finite position";
                    }
                } else if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ) {
                    badReads++;
                    consecutiveBadReads++;
                    lastReadError = "I2C read rejected; data is stale";
                } else {
                    lastReadError = "Pinpoint is not READY: " + status;
                }
            } catch (RuntimeException error) {
                badReads++;
                consecutiveBadReads++;
                lastReadError = error.getClass().getSimpleName();
            }
            long nowMs = System.currentTimeMillis();
            if (nowMs - lastLogMs >= LOG_INTERVAL_MS) {
                lastLogMs = nowMs;
                RobotLog.ii("DEADWHEEL", "status=%s %s | drift=%s | reads good/bad=%d/%d | %s",
                        lastStatusText, lastPoseText, driftResult, goodReads, badReads,
                        lastReadError);
            }
            previousA = gamepad1.a;
            previousY = gamepad1.y;
            telemetry.addData("Good / bad reads", "%d / %d", goodReads, badReads);
            telemetry.addData("Last read issue", lastReadError);
            if (consecutiveBadReads >= 20) {
                telemetry.addLine("Persistent I2C error: check Pinpoint power, I2C cable,");
                telemetry.addLine("hub port, and configured device type/name. Stop tuning.");
            }
            telemetry.addLine("A: zero deltas; STOP: finish");
            telemetry.update();
            sleep(100); // 10 Hz is plenty for a hand-pushed direction test.
        }
    }
}
