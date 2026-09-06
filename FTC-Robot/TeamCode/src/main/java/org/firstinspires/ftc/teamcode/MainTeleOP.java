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
 * DRIVE
 * Left Stick Y     → Forward / Backward
 * Left Stick X     → Strafe Left / Right
 * Right Stick X    → Turn Left / Right
 * (Touching any stick cancels an active navigation)
 *
 * ODOMETRY (sticks neutral)
 * D-Pad Up/Down    → Target Y  ± 0.0254 m (1 in)
 * D-Pad Left/Right → Target X  ± 0.0254 m (1 in)
 * X / B            → Target Heading −5° / +5°
 * Y                → Navigate to target (turn, then drive)
 * Left Bumper      → Snap target to current pose
 * Back             → Reset odometry pose + IMU (robot must be still)
 * ─────────────────────────────────────────────────────────────
 */

@TeleOp(name = "Pineapple", group = "Competition")
public class MainTeleOP extends OpMode {

    private static final double NOMINAL_VOLTAGE   = 12.0;
    private static final double MIN_VALID_VOLTAGE = 8.0;
    private static final double MAX_VOLTAGE_SCALE = 1.15;
    private static final double STICK_DEADBAND    = 0.1;

    private Drivetrain drivetrain;
    private Odometry odometry;

    private VoltageSensor voltageSensor;
    private double batteryVoltage = NOMINAL_VOLTAGE;
    private double voltageScale = 1.0;
    private boolean validVoltage;

    private final Gamepad currentGamepad = new Gamepad();
    private final Gamepad previousGamepad = new Gamepad();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopHz = 0.0;


    // ─────────────────────────────────────────────────────────
    // MAIN (LinearOpMode lifecycle)
    // ─────────────────────────────────────────────────────────

    @Override
    public void init() {
        enableBulkReads();
        initializeVoltageSensor();

        drivetrain = new Drivetrain(hardwareMap);
        odometry   = new Odometry(hardwareMap, drivetrain);
        odometry.init();   // robot must be stationary here (IMU calibration)

        telemetry.addLine("PINEAPPLE READY 🍍 (METRIC)");
        telemetry.addLine("Drivetrain & Odometry initialized");
        telemetry.addData("Battery", "%.2f V", batteryVoltage);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        updateBatteryVoltage();
        odometry.update();
        telemetry.addLine("Waiting for START...");
        telemetry.addData("Battery", "%.2f V", batteryVoltage);
        telemetry.addData("Pinpoint", odometry.getStatus());
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

        // 1. One odometry read per loop (I2C – don't read again elsewhere)
        odometry.update();

        // 2. Driver override?
        boolean isDriving = Math.abs(currentGamepad.left_stick_y)  > STICK_DEADBAND
                         || Math.abs(currentGamepad.left_stick_x)  > STICK_DEADBAND
                         || Math.abs(currentGamepad.right_stick_x) > STICK_DEADBAND;

        if (isDriving) {
            odometry.cancelNavigation();            // driver always wins
            drivetrain.update(currentGamepad, voltageScale);
        } else {
            // 3. Button edges (rising-edge detection done here, once)
            if (pressed(currentGamepad.back, previousGamepad.back)) {
                odometry.reset();
            }
            if (pressed(currentGamepad.left_bumper, previousGamepad.left_bumper)) {
                odometry.syncTargetsToPose();
            }

            odometry.processInputs(
                pressed(currentGamepad.dpad_up,    previousGamepad.dpad_up),
                pressed(currentGamepad.dpad_down,  previousGamepad.dpad_down),
                pressed(currentGamepad.dpad_left,  previousGamepad.dpad_left),
                pressed(currentGamepad.dpad_right, previousGamepad.dpad_right),
                pressed(currentGamepad.x, previousGamepad.x),
                pressed(currentGamepad.b, previousGamepad.b),
                pressed(currentGamepad.y, previousGamepad.y)
            );

            // 4. Navigation step (returns true when idle/finished)
            if (odometry.updateNavigation(voltageScale)) {
                drivetrain.drive(0, 0, 0, voltageScale);
            }
        }

        sendTelemetry();
    }

    @Override
    public void stop() {
        odometry.cancelNavigation();
        drivetrain.drive(0, 0, 0, 1.0);
    }

    // ─────────────────────────────────────────────────────────

    private static boolean pressed(boolean now, boolean before) {
        return now && !before;
    }

    private void enableBulkReads() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    private void initializeVoltageSensor() {
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
            updateBatteryVoltage();
        }
    }

    private void updateBatteryVoltage() {
        if (voltageSensor == null) {
            batteryVoltage = NOMINAL_VOLTAGE;
            voltageScale = 1.0;

            return;
        }
        batteryVoltage = Math.max(voltageSensor.getVoltage(), MIN_VALID_VOLTAGE);
        voltageScale = Range.clip(NOMINAL_VOLTAGE / batteryVoltage, 1.0, MAX_VOLTAGE_SCALE);
    }


    // ─────────────────────────────────────────────────────────
    // GAMEPAD
    // ─────────────────────────────────────────────────────────

    private void updateGamepad() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);
    }

    private void calculateLoopSpeed() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt > 1e-6) loopHz = 1.0 / dt;
    }

    private void sendTelemetry() {
        telemetry.addLine("──────── PINEAPPLE 🍍 ────────");
        telemetry.addData("Loop Speed", "%.0f Hz", loopHz);
        telemetry.addData("Battery", "%.2f V", batteryVoltage);
        telemetry.addData("Voltage Scale", "%.3f", voltageScale);
        telemetry.addLine();
        drivetrain.addTelemetry(telemetry);

        telemetry.addLine();
        telemetry.addData("Pos (X,Y) m", "(%.3f, %.3f)", odometry.getX(), odometry.getY());
        telemetry.addData("Heading deg", "%.1f", odometry.getHeadingDeg());
        telemetry.addData("Target (X,Y) m", "(%.3f, %.3f)", odometry.getTargetX(), odometry.getTargetY());
        telemetry.addData("Target Heading deg", "%.1f", odometry.getTargetHeading());
        telemetry.addData("Navigating", odometry.isNavigating() ? "YES" : "no");

        telemetry.update();
    }
}
