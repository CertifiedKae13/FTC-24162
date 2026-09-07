package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * ─────────────────────────────────────────────────────────────
 *                      PINEAPPLE TELEOP
 * ─────────────────────────────────────────────────────────────
 *
 * CONTROLS
 *
 * Left Stick Y   → Forward / Backward
 * Left Stick X   → Strafe Left / Right
 * Right Stick X  → Turn Left / Right
 *
 * D-pad          → build a target position  (10 cm per press)
 * X / B          → nudge target heading     (5° per press)
 * A              → execute turn to target heading
 * Y              → drive to target position
 *
 * Drivetrain logic is handled by Drivetrain.java
 * Odometry logic  is handled by Odometry.java
 * ─────────────────────────────────────────────────────────────
 */

@TeleOp(name = "Pineapple", group = "Competition")
public class MainTeleOP extends LinearOpMode {

    // ─────────────────────────────────────────────────────────
    // VOLTAGE COMPENSATION
    // ─────────────────────────────────────────────────────────

    private static final double NOMINAL_VOLTAGE = 12.0;

    // Prevents crazy compensation if battery voltage gets very low.
    private static final double MIN_VALID_VOLTAGE = 8.0;

    // Keeps voltage compensation from becoming too aggressive.
    private static final double MAX_VOLTAGE_SCALE = 1.15;


    // ─────────────────────────────────────────────────────────
    // SUBSYSTEMS
    // ─────────────────────────────────────────────────────────

    private Drivetrain drivetrain;
    private Odometry odometry;


    // ─────────────────────────────────────────────────────────
    // HARDWARE / STATE
    // ─────────────────────────────────────────────────────────

    private VoltageSensor voltageSensor;

    private double batteryVoltage = NOMINAL_VOLTAGE;
    private double voltageScale = 1.0;

    private final Gamepad currentGamepad = new Gamepad();
    private final Gamepad previousGamepad = new Gamepad();

    private final ElapsedTime loopTimer = new ElapsedTime();

    private double loopHz = 0.0;


    // ─────────────────────────────────────────────────────────
    // MAIN (LinearOpMode lifecycle)
    // ─────────────────────────────────────────────────────────

    @Override
    public void runOpMode() {

        enableBulkReads();

        initializeVoltageSensor();

        drivetrain = new Drivetrain(hardwareMap);

        odometry = new Odometry(hardwareMap, drivetrain);
        odometry.init();

        telemetry.addLine("PINEAPPLE READY 🍍");
        telemetry.addLine("Drivetrain initialized successfully");
        telemetry.addLine("Odometry initialized");
        telemetry.addData("Battery", "%.2f V", batteryVoltage);
        telemetry.update();

        // Wait for the driver to press PLAY (shows init telemetry meanwhile).
        while (!isStarted() && !isStopRequested()) {

            updateBatteryVoltage();

            telemetry.addLine("Waiting for START...");
            telemetry.addData("Battery", "%.2f V", batteryVoltage);
            telemetry.addData("Voltage Scale", "%.2f", voltageScale);
            telemetry.update();
        }

        // Snapshot the gamepad state at the moment the match starts.
        loopTimer.reset();
        currentGamepad.copy(gamepad1);
        previousGamepad.copy(gamepad1);

        // Main loop.
        while (opModeIsActive()) {

            calculateLoopSpeed();

            updateGamepad();

            updateBatteryVoltage();

            odometry.setVoltageScale(voltageScale);
            odometry.update();
            odometry.processPuzzleKeys(gamepad1);

            if (odometry.isNavigatingToTarget()) {
                odometry.navigateToTarget(this);
            } else {
                drivetrain.update(
                        currentGamepad,
                        voltageScale
                );
            }

            sendTelemetry();
        }

        /*
         * Sending a zeroed Gamepad forces drivetrain motors
         * to receive 0 power before the OpMode completely stops.
         */
        drivetrain.update(new Gamepad(), 1.0);

        telemetry.addLine("Pineapple stopped.");
        telemetry.update();
    }


    // ─────────────────────────────────────────────────────────
    // BULK READING
    // ─────────────────────────────────────────────────────────

    private void enableBulkReads() {

        for (LynxModule hub :
                hardwareMap.getAll(LynxModule.class)) {

            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.AUTO
            );
        }
    }


    // ─────────────────────────────────────────────────────────
    // VOLTAGE SENSOR
    // ─────────────────────────────────────────────────────────

    private void initializeVoltageSensor() {

        if (hardwareMap.voltageSensor
                .iterator()
                .hasNext()) {

            voltageSensor =
                    hardwareMap.voltageSensor
                            .iterator()
                            .next();

            batteryVoltage =
                    voltageSensor.getVoltage();
        }
    }


    private void updateBatteryVoltage() {

        if (voltageSensor == null) {

            batteryVoltage = NOMINAL_VOLTAGE;
            voltageScale = 1.0;

            return;
        }

        double measuredVoltage =
                voltageSensor.getVoltage();

        /*
         * Protect against invalid / extremely low readings.
         */
        batteryVoltage =
                Math.max(
                        measuredVoltage,
                        MIN_VALID_VOLTAGE
                );

        /*
         * Example:
         *
         * 12.0V battery:
         * 12 / 12 = 1.00
         *
         * 11.5V battery:
         * 12 / 11.5 = 1.043
         *
         * Scale is capped so we don't overcompensate.
         */

        voltageScale =
                Range.clip(
                        NOMINAL_VOLTAGE / batteryVoltage,
                        1.0,
                        MAX_VOLTAGE_SCALE
                );
    }


    // ─────────────────────────────────────────────────────────
    // GAMEPAD
    // ─────────────────────────────────────────────────────────

    private void updateGamepad() {

        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);
    }


    // ─────────────────────────────────────────────────────────
    // LOOP SPEED
    // ─────────────────────────────────────────────────────────

    private void calculateLoopSpeed() {

        double dt = loopTimer.seconds();

        loopTimer.reset();

        if (dt > 0.000001) {
            loopHz = 1.0 / dt;
        }
    }


    // ─────────────────────────────────────────────────────────
    // TELEMETRY
    // ─────────────────────────────────────────────────────────

    private void sendTelemetry() {

        telemetry.addLine("──────── PINEAPPLE 🍍 ────────");

        telemetry.addData(
                "Loop Speed",
                "%.0f Hz",
                loopHz
        );

        telemetry.addData(
                "Battery",
                "%.2f V",
                batteryVoltage
        );

        telemetry.addData(
                "Voltage Scale",
                "%.3f",
                voltageScale
        );

        telemetry.addLine();

        drivetrain.addTelemetry(telemetry);

        telemetry.addLine();
        telemetry.addData("Distance Coeff", "%.3f", odometry.getDistanceCoefficient());
        telemetry.addData("Pos (X,Y) cm", "(%.1f, %.1f)", odometry.getX(), odometry.getY());
        telemetry.addData("Heading deg", "%.1f", odometry.getHeadingDeg());
        telemetry.addData("Target (X,Y) cm", "(%.1f, %.1f)", odometry.getTargetX(), odometry.getTargetY());
        telemetry.addData("Target Heading deg", "%.1f", odometry.getTargetHeading());
        telemetry.addData("Navigating", odometry.isNavigatingToTarget() ? "YES" : "no");

        telemetry.update();
    }
}
