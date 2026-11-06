package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * PINEAPPLE TELEOP: inches / degrees, field-relative (origin = field center, see FieldStart).
 * Sticks cancel navigation; release rebases the navigation target to current pose.
 * Manual translation holds heading when Pinpoint is ready and turn stick is neutral.
 * Mechanisms (gamepad 1): A = intake in on/off, LB = intake out on/off (starting one stops the other); L2 (left
 * trigger) = launch1, R2 (right trigger) = launch2, power follows how far the trigger is pressed.
 * D-pad: target 1 inch along FIELD axes (up/down = Y, right/left = X). Gamepad 2 RB / gamepad 1 B: heading CCW/CW by 5 degrees. X: toggle home-hold (return to field center if moved). While ON it overrides all targets; sticks pause it, only X turns it off.
 * Idle target follows the pose; a set target is held.
 * Y: turn then navigate. Gamepad 2 LB: cancel + sync. Back: stop, settle, reset/calibrate.
 * Keep the robot stationary during INIT and calibration.
 */
@TeleOp(name = "Pineapple2", group = "Competition")
public class MainTeleOP2 extends OpMode {
    private static final boolean DEBUG_TELEMETRY = false;
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double MIN_VALID_VOLTAGE = 8.0;
    private static final double MAX_VALID_VOLTAGE = 16.8;
    private static final double MIN_VOLTAGE_SCALE = 0.80;
    private static final double MAX_VOLTAGE_SCALE = 1.15;
    private static final double STICK_DEADBAND = 0.10;
    /** TeleOp only: no wheel ever gets more than this power (stops the robot tipping back on hard stops). */
    private static final double MAX_TELEOP_SPEED = 0.5;
    // Mechanisms: names in the robot configuration. A motor that is not configured is skipped (no crash).
    private static final String INTAKE_NAME = "intake";
    private static final String LAUNCH1_NAME = "launch1";
    private static final String LAUNCH2_NAME = "launch2";
    private static final double INTAKE_POWER = 1.0;      // flip the sign if it spins the wrong way
    private static final double LAUNCH1_SIGN = 1.0;      // flip to -1.0 if launch1 spins the wrong way
    private static final double LAUNCH2_SIGN = 1.0;      // flip to -1.0 if launch2 spins the wrong way
    private static final double TRIGGER_DEADBAND = 0.05;
    private static final double BATTERY_INTERVAL_S = 0.20;
    private static final double TELEMETRY_INTERVAL_S = 0.20;
    private Drivetrain2 drivetrain;
    private DcMotor intake, launch1, launch2;
    private int intakeState;   // +1 in, -1 out, 0 off (toggled by A / LB)
    private Odometry2 odometry;
    private TelemetryCapture2 telemetryCapture;
    private FtcDashboard dashboard;
    private VoltageSensor voltageSensor;
    private double batteryVoltage = Double.NaN;
    private double filteredVoltage = Double.NaN;
    private double voltageScale = 1.0;
    private boolean validVoltage;
    private boolean manualTargetsNeedSync;
    private boolean started;
    private boolean wasDriving;
    private double brakeUntil;
    private static final double RELEASE_BRAKE_S = 0.5;   // max time to actively brake after the sticks are released
    private String driveMode = "INIT";
    private final Gamepad currentGamepad = new Gamepad();
    private final Gamepad previousGamepad = new Gamepad();
    private final Gamepad currentGamepad2 = new Gamepad();
    private final Gamepad previousGamepad2 = new Gamepad();
    private final ElapsedTime clock = new ElapsedTime();
    private double lastBatteryTime = -1.0;
    private double lastTelemetryTime = -1.0;
    private double lastLoopTime;

    @Override
    public void init() {
        // Keep Driver Station telemetry and Dashboard telemetry separate.
        // Dashboard gets dedicated numeric TelemetryPackets so every controller
        // value can be selected and graphed without formatted-string parsing.
        dashboard = FtcDashboard.getInstance();
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        }
        updateBatteryVoltage(true);
        drivetrain = new Drivetrain2(hardwareMap);
        drivetrain.setMaxWheelPower(MAX_TELEOP_SPEED);   // covers sticks, Y/A navigation and release braking
        intake = mechanism(INTAKE_NAME, DcMotor.ZeroPowerBehavior.BRAKE);
        launch1 = mechanism(LAUNCH1_NAME, DcMotor.ZeroPowerBehavior.FLOAT);   // flywheels coast down
        launch2 = mechanism(LAUNCH2_NAME, DcMotor.ZeroPowerBehavior.FLOAT);
        stopDrive();
        odometry = new Odometry2(hardwareMap, drivetrain);
        odometry.init();
        telemetryCapture = new TelemetryCapture2();
        telemetry.setMsTransmissionInterval(200);
        sendTelemetry(true);
    }

    @Override
    public void init_loop() {
        updateBatteryVoltage(false);
        odometry.update();
        odometry.updateReset();
        stopDrive();
        if (telemetryCapture != null) {
            telemetryCapture.sample(clock.seconds(), "INIT", currentGamepad,
                    drivetrain, odometry, batteryVoltage, voltageScale);
        }
        sendTelemetry(false);
    }

    @Override
    public void start() {
        lastLoopTime = clock.seconds();
        manualTargetsNeedSync = false;
        started = true;
        driveMode = "IDLE";
        currentGamepad.copy(gamepad1);
        previousGamepad.copy(currentGamepad);
        currentGamepad2.copy(gamepad2);
        previousGamepad2.copy(currentGamepad2);
        stopDrive();
    }

    @Override
    public void loop() {
        previousGamepad.copy(currentGamepad);
        currentGamepad.copy(gamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
        updateBatteryVoltage(false);
        updateMechanisms();
        // Tell odometry whether the driver is on the sticks BEFORE it may start an automatic return.
        odometry.setManualDriving(Math.abs(currentGamepad.left_stick_y) > STICK_DEADBAND
                || Math.abs(currentGamepad.left_stick_x) > STICK_DEADBAND
                || Math.abs(currentGamepad.right_stick_x) > STICK_DEADBAND);
        odometry.update();
        // Measure after the sensor poll so its latency is included in this step.
        double now = clock.seconds();
        double dt = now - lastLoopTime;
        lastLoopTime = now;
        boolean driving = Math.abs(currentGamepad.left_stick_y) > STICK_DEADBAND
                || Math.abs(currentGamepad.left_stick_x) > STICK_DEADBAND
                || Math.abs(currentGamepad.right_stick_x) > STICK_DEADBAND;

        if (wasDriving && !driving) {                 // sticks just released: stop, don't slide
            brakeUntil = now + RELEASE_BRAKE_S;
            odometry.resetBrake();
        }
        wasDriving = driving;
        if (!driving) drivetrain.resetManualHeadingHold();
        if (driving) {
            odometry.cancelPendingReset();
            manualTargetsNeedSync = true;
        } else if (manualTargetsNeedSync && odometry.isReady()) {
            odometry.syncTargetsToPose();
            manualTargetsNeedSync = false;
        }
        if (odometry.isResetting()) {
            odometry.updateReset();
            stopDrive();
            driveMode = "RESETTING";
        } else if (!odometry.canDriveManually()) {
            stopDrive();
            driveMode = "WAITING FOR CALIBRATION";
        } else if (driving) {
            odometry.cancelNavigation();
            drivetrain.update(currentGamepad, voltageScale,
                    odometry.isReady() ? odometry.getHeadingDeg() : Double.NaN, dt);
            driveMode = "MANUAL / " + drivetrain.getManualAssistStatus();
        } else if (pressed(currentGamepad.back, previousGamepad.back)) {
            odometry.reset();
            stopDrive();
            driveMode = "RESETTING";
        } else if (pressed(currentGamepad2.left_bumper, previousGamepad2.left_bumper)) {   // gamepad 1 LB = intake now
            odometry.syncTargetsToPose();
            stopDrive();
            driveMode = "IDLE";
        } else {
            if (pressed(currentGamepad.x, previousGamepad.x)) odometry.toggleHomeHold();
            odometry.processInputs(
                    pressed(currentGamepad.dpad_up, previousGamepad.dpad_up),
                    pressed(currentGamepad.dpad_down, previousGamepad.dpad_down),
                    pressed(currentGamepad.dpad_left, previousGamepad.dpad_left),
                    pressed(currentGamepad.dpad_right, previousGamepad.dpad_right),
                    pressed(currentGamepad2.right_bumper, previousGamepad2.right_bumper), // heading -5 (gamepad 1 RB is free)
                    pressed(currentGamepad.b, previousGamepad.b),
                    pressed(currentGamepad.y, previousGamepad.y));
            boolean braking = false;
            if (odometry.updateNavigation(voltageScale)) {
                braking = now < brakeUntil && odometry.activeBrake(voltageScale);
                if (!braking) { brakeUntil = 0; stopDrive(); }
            }
            driveMode = odometry.isNavigating() ? "NAVIGATING" : braking ? "BRAKING" : "IDLE";
        }
        if (telemetryCapture != null) {
            telemetryCapture.sample(now, driveMode, currentGamepad,
                    drivetrain, odometry, batteryVoltage, voltageScale);
        }
        sendTelemetry(false);
    }

    @Override
    public void stop() {
        try {
            if (odometry != null) odometry.cancelNavigation();
        } finally {
            stopDrive();
            stopMechanisms();
            if (telemetryCapture != null) telemetryCapture.close();
        }
    }

    private void stopDrive() {
        if (drivetrain != null) drivetrain.stop();
    }

    /** The configured motor, or null (with a telemetry note) if it is not in the robot configuration. */
    private DcMotor mechanism(String name, DcMotor.ZeroPowerBehavior zero) {
        DcMotor m = hardwareMap.tryGet(DcMotor.class, name);
        if (m == null) return null;
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(zero);
        m.setPower(0.0);
        return m;
    }

    /** Intake toggles on A (in) / LB (out), launch1 / launch2 on the left / right triggers (analog). Runs every loop. */
    private void updateMechanisms() {
        // Toggles: a press starts that direction (switching off the other); pressing the same button again stops it.
        if (pressed(currentGamepad.a, previousGamepad.a)) intakeState = intakeState == 1 ? 0 : 1;
        if (pressed(currentGamepad.left_bumper, previousGamepad.left_bumper)) intakeState = intakeState == -1 ? 0 : -1;
        setPower(intake, intakeState * INTAKE_POWER);
        setPower(launch1, LAUNCH1_SIGN * trigger(currentGamepad.left_trigger));
        setPower(launch2, LAUNCH2_SIGN * trigger(currentGamepad.right_trigger));
    }

    private void stopMechanisms() {
        intakeState = 0;
        setPower(intake, 0.0);
        setPower(launch1, 0.0);
        setPower(launch2, 0.0);
    }

    private static double trigger(float value) {
        return value > TRIGGER_DEADBAND ? Range.clip(value, 0.0, 1.0) : 0.0;
    }

    private static void setPower(DcMotor m, double power) {
        if (m != null) m.setPower(power);
    }

    private static String motorState(DcMotor m) {
        return m == null ? "NOT CONFIGURED" : String.format("%.2f", m.getPower());
    }

    private static boolean pressed(boolean now, boolean before) {
        return now && !before;
    }

    private void updateBatteryVoltage(boolean force) {
        double now = clock.seconds();
        if (!force && now - lastBatteryTime < BATTERY_INTERVAL_S) return;
        lastBatteryTime = now;
        try {
            batteryVoltage = voltageSensor == null ? Double.NaN : voltageSensor.getVoltage();
        } catch (RuntimeException ex) {
            batteryVoltage = Double.NaN;
        }
        validVoltage = !Double.isNaN(batteryVoltage) && !Double.isInfinite(batteryVoltage)
                && batteryVoltage >= MIN_VALID_VOLTAGE && batteryVoltage <= MAX_VALID_VOLTAGE;
        if (!validVoltage) {
            filteredVoltage = Double.NaN;
            voltageScale = 1.0;
            return;
        }
        filteredVoltage = Double.isNaN(filteredVoltage) ? batteryVoltage
                : 0.75 * filteredVoltage + 0.25 * batteryVoltage;
        voltageScale = Range.clip(NOMINAL_VOLTAGE / filteredVoltage,
                MIN_VOLTAGE_SCALE, MAX_VOLTAGE_SCALE);
    }

    private void sendTelemetry(boolean force) {
        double now = clock.seconds();
        if (!force && now - lastTelemetryTime < TELEMETRY_INTERVAL_S) return;
        lastTelemetryTime = now;

        sendDashboardTelemetry(now);

        // This clear only affects the Driver Station now. It no longer clears
        // FTC Dashboard telemetry, which prevents the dashboard from blinking.
        telemetry.clearAll();
        telemetry.addData("Mode", !started ? "INIT / KEEP STILL" : driveMode);
        telemetry.addData("Pinpoint", "%s / %s", odometry.getStatus(),
                odometry.isReady() ? "READY" : "NOT READY");
        telemetry.addData("Intake / launch1 / launch2", "%s / %s / %s",
                motorState(intake), motorState(launch1), motorState(launch2));
        if (validVoltage) telemetry.addData("Battery", "%.1f V", batteryVoltage);
        else telemetry.addData("Battery", "Unavailable / compensation off");
        if (telemetryCapture != null) {
            telemetryCapture.addDriverStationTelemetry(
                    telemetry, driveMode, currentGamepad, drivetrain, odometry);
        } else {
            telemetry.addData("Pose", "X %.2f in  Y %.2f in  H %.1f deg (0-360)",
                    odometry.getX(), odometry.getY(), odometry.getHeadingDeg());
            telemetry.addData("Target", "X %.2f in  Y %.2f in  H %.1f deg (0-360)",
                    odometry.getTargetX(), odometry.getTargetY(), odometry.getTargetHeading());
            telemetry.addData("Last action", odometry.getLastResult());
        }
        if (!started) {
            telemetry.addLine("A: intake in on/off | LB: intake out on/off | L2: launch1 | R2: launch2");
            telemetry.addLine("D-pad: 1 in (field X/Y) | GP2 RB / B: 5 deg | X: home-hold on/off (locks targets) | Y: GO | GP2 LB: cancel+sync | Back: reset");
        }
        if (DEBUG_TELEMETRY) {
            drivetrain.addTelemetry(telemetry);
            odometry.addTelemetry(telemetry);
        }
        telemetry.update();
    }

    /**
     * Dashboard-only telemetry. Numeric controller quantities are always sent
     * as raw doubles (never formatted strings), so Dashboard can graph them.
     */
    private void sendDashboardTelemetry(double now) {
        if (dashboard == null || drivetrain == null || odometry == null) return;

        TelemetryPacket packet = new TelemetryPacket();

        // Time / power
        packet.put("time_s", now);
        packet.put("battery_v", validVoltage ? batteryVoltage : Double.NaN);
        packet.put("voltage_scale", voltageScale);

        // Pose and requested pose
        packet.put("pose_x_in", odometry.getX());
        packet.put("pose_y_in", odometry.getY());
        packet.put("heading_deg", odometry.getHeadingDeg());
        packet.put("home_hold", odometry.isHomeHold() ? 1 : 0);
        packet.put("home_hold_state", odometry.getHomeHoldState());
        packet.put("target_x_in", odometry.getTargetX());
        packet.put("target_y_in", odometry.getTargetY());
        packet.put("target_heading_deg", odometry.getTargetHeading());

        // Navigation error
        packet.put("error_x_in", odometry.getErrorX());
        packet.put("error_y_in", odometry.getErrorY());
        packet.put("distance_error_in", odometry.getDistanceError());
        packet.put("heading_error_deg", odometry.getHeadingErrorDeg());

        // Measured motion
        packet.put("velocity_x_ips", odometry.getVelocityX());
        packet.put("velocity_y_ips", odometry.getVelocityY());
        packet.put("angular_velocity_dps", odometry.getAngularVelocityDegPerSec());

        // Field-frame controller command
        packet.put("field_command_x", odometry.getCommandX());
        packet.put("field_command_y", odometry.getCommandY());

        // Robot-frame controller command
        packet.put("robot_forward_cmd", drivetrain.getRequestedForward());
        packet.put("robot_right_cmd", drivetrain.getRequestedRight());
        packet.put("robot_cw_cmd", drivetrain.getRequestedClockwise());

        // Individual wheel powers
        packet.put("front_left_power", drivetrain.getFrontLeftPower());
        packet.put("front_right_power", drivetrain.getFrontRightPower());
        packet.put("back_left_power", drivetrain.getBackLeftPower());
        packet.put("back_right_power", drivetrain.getBackRightPower());

        // Turn controller internals
        packet.put("turn_p", odometry.getTurnP());
        packet.put("turn_d", odometry.getTurnD());
        packet.put("turn_ff", odometry.getTurnFeedforward());
        packet.put("turn_raw", odometry.getTurnRaw());
        packet.put("turn_ccw_out", odometry.getTurnCommand());
        packet.put("control_dt_s", odometry.getControlDt());
        packet.put("turn_clipped", odometry.isTurnClipped() ? 1.0 : 0.0);

        // Driver inputs, useful for separating manual commands from controller output.
        packet.put("stick_left_y", currentGamepad.left_stick_y);
        packet.put("stick_left_x", currentGamepad.left_stick_x);
        packet.put("stick_right_x", currentGamepad.right_stick_x);

        // State flags as 0/1 so they can be graphed alongside controller values.
        packet.put("odometry_ready", odometry.isReady() ? 1.0 : 0.0);
        packet.put("motion_estimate_usable", odometry.isMotionEstimateUsable() ? 1.0 : 0.0);
        packet.put("navigating", odometry.isNavigating() ? 1.0 : 0.0);

        // Text is included only for at-a-glance status; all quantities above are numeric.
        packet.put("mode", !started ? "INIT / KEEP STILL" : driveMode);
        packet.put("pinpoint_status", odometry.getStatus());
        packet.put("nav_state", odometry.getNavigationState());
        packet.put("last_result", odometry.getLastResult());
        packet.put("log_file", telemetryCapture == null ? "" : telemetryCapture.getFileName());

        dashboard.sendTelemetryPacket(packet);
    }
}
