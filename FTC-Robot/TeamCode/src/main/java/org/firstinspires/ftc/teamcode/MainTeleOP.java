package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
 * Drivetrain logic is handled by Drivetrain.java
 * ─────────────────────────────────────────────────────────────
 */

@TeleOp(name = "Pineapple", group = "Competition")
public class MainTeleOP extends OpMode {

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
    // INIT
    // ─────────────────────────────────────────────────────────

    @Override
    public void init() {

        enableBulkReads();

        initializeVoltageSensor();

        drivetrain = new Drivetrain(hardwareMap);

        telemetry.addLine("PINEAPPLE READY 🍍");
        telemetry.addLine("Drivetrain initialized successfully");
        telemetry.addData("Battery", "%.2f V", batteryVoltage);
        telemetry.update();
    }


    // ─────────────────────────────────────────────────────────
    // INIT LOOP
    // Runs while waiting for PLAY
    // ─────────────────────────────────────────────────────────

    @Override
    public void init_loop() {

        updateBatteryVoltage();

        telemetry.addLine("Waiting for START...");
        telemetry.addData("Battery", "%.2f V", batteryVoltage);
        telemetry.addData("Voltage Scale", "%.2f", voltageScale);
        telemetry.update();
    }


    // ─────────────────────────────────────────────────────────
    // START
    // ─────────────────────────────────────────────────────────

    @Override
    public void start() {

        loopTimer.reset();

        currentGamepad.copy(gamepad1);
        previousGamepad.copy(gamepad1);
    }


    // ─────────────────────────────────────────────────────────
    // MAIN LOOP
    // ─────────────────────────────────────────────────────────

    @Override
    public void loop() {

        calculateLoopSpeed();

        updateGamepad();

        updateBatteryVoltage();

        drivetrain.update(
                currentGamepad,
                voltageScale
        );

        sendTelemetry();
    }


    // ─────────────────────────────────────────────────────────
    // STOP
    // ─────────────────────────────────────────────────────────

    @Override
    public void stop() {

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

        telemetry.update();
    }
}