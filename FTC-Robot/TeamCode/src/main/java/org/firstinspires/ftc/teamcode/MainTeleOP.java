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
 *                      PINEAPPLE TELEOP (METRIC)
 * ─────────────────────────────────────────────────────────────
 *
 * CONTROLS
 * Left Stick Y   → Forward / Backward
 * Left Stick X   → Strafe Left / Right
 * Right Stick X  → Turn Left / Right
 *
 * ODOMETRY CONTROLS (When sticks are neutral)
 * D-Pad Up/Down  → Adjust Target Y (+/- 0.0254m)
 * D-Pad Left/Right → Adjust Target X (+/- 0.0254m)
 * X Button       → Target Heading -5°
 * B Button       → Target Heading +5°
 * Y Button       → Navigate to Target
 * ─────────────────────────────────────────────────────────────
 */

@TeleOp(name = "Pineapple", group = "Competition")
public class MainTeleOP extends OpMode {

    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double MIN_VALID_VOLTAGE = 8.0;
    private static final double MAX_VOLTAGE_SCALE = 1.15;

    private Drivetrain drivetrain;
    private Odometry odometry;

    private VoltageSensor voltageSensor;
    private double batteryVoltage = NOMINAL_VOLTAGE;
    private double voltageScale = 1.0;

    private final Gamepad currentGamepad = new Gamepad();
    private final Gamepad previousGamepad = new Gamepad();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopHz = 0.0;

    @Override
    public void init() {
        enableBulkReads();
        initializeVoltageSensor();

        drivetrain = new Drivetrain(hardwareMap);
        odometry = new Odometry(hardwareMap, drivetrain);
        odometry.init();

        telemetry.addLine("PINEAPPLE READY 🍍 (METRIC)");
        telemetry.addLine("Drivetrain & Odometry initialized");
        telemetry.addData("Battery", "%.2f V", batteryVoltage);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        updateBatteryVoltage();
        telemetry.addLine("Waiting for START...");
        telemetry.addData("Battery", "%.2f V", batteryVoltage);
        telemetry.update();
    }

    @Override
    public void start() {
        loopTimer.reset();
        currentGamepad.copy(gamepad1);
        previousGamepad.copy(gamepad1);
    }

    @Override
    public void loop() {
        calculateLoopSpeed();
        updateGamepad();
        updateBatteryVoltage();

        // 1. Update Odometry Position
        odometry.update();

        // 2. Check if we are manually driving
        boolean isDriving = Math.abs(currentGamepad.left_stick_y) > 0.1 || 
                            Math.abs(currentGamepad.left_stick_x) > 0.1 || 
                            Math.abs(currentGamepad.right_stick_x) > 0.1;

        if (isDriving) {
            // Normal TeleOp Control
            drivetrain.update(currentGamepad, voltageScale);
            // If we interrupt navigation by driving, stop navigation state
            // (Optional: remove this line if you want navigation to resume after manual drive)
            // odometry.resetNavigation(); 
        } else {
            // 3. Process Odometry Inputs (Only when not driving)
            odometry.processInputs(
                currentGamepad.dpad_up, currentGamepad.dpad_down,
                currentGamepad.dpad_left, currentGamepad.dpad_right,
                currentGamepad.x, currentGamepad.b, currentGamepad.y
            );

            // 4. Execute Navigation Step
            boolean navComplete = odometry.updateNavigation();
            
            if (navComplete) {
                // Ensure motors stop if navigation is done/idle
                drivetrain.drive(0, 0, 0, voltageScale);
            }
        }

        sendTelemetry();
    }

    @Override
    public void stop() {
        drivetrain.update(new Gamepad(), 1.0);
        telemetry.addLine("Pineapple stopped.");
        telemetry.update();
    }

    private void enableBulkReads() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    private void initializeVoltageSensor() {
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            batteryVoltage = voltageSensor.getVoltage();
        }
    }

    private void updateBatteryVoltage() {
        if (voltageSensor == null) {
            batteryVoltage = NOMINAL_VOLTAGE;
            voltageScale = 1.0;
            return;
        }
        double measuredVoltage = voltageSensor.getVoltage();
        batteryVoltage = Math.max(measuredVoltage, MIN_VALID_VOLTAGE);
        voltageScale = Range.clip(NOMINAL_VOLTAGE / batteryVoltage, 1.0, MAX_VOLTAGE_SCALE);
    }

    private void updateGamepad() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);
    }

    private void calculateLoopSpeed() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt > 0.000001) loopHz = 1.0 / dt;
    }

    private void sendTelemetry() {
        telemetry.addLine("──────── PINEAPPLE 🍍 ────────");
        telemetry.addData("Loop Speed", "%.0f Hz", loopHz);
        telemetry.addData("Battery", "%.2f V", batteryVoltage);
        telemetry.addData("Voltage Scale", "%.3f", voltageScale);
        telemetry.addLine();
        
        drivetrain.addTelemetry(telemetry);
        odometry.addTelemetry(telemetry);
        
        telemetry.update();
    }
}