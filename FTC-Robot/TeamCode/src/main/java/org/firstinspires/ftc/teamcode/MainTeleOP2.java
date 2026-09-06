package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
* PINEAPPLE TELEOP -- metres / degrees.
* Sticks: manual drive, cancelling navigation and any reset not yet issued.
* Neutral sticks: D-pad nudges the target 1 inch in the robot's CURRENT frame.
* X / B: target heading left (CCW) / right (CW), 5 degrees.
* Y: turn, then navigate. Left bumper: cancel and snap target to current pose.
* Back: stop, settle, then reset pose and recalibrate the Pinpoint IMU.
* Once calibration starts, drive is held at zero until it completes or times out.
* Keep the robot stationary during INIT and calibration.
*/
@TeleOp(name = "Pineapple2", group = "Competition")
public class MainTeleOP2 extends OpMode {
   private static final double NOMINAL_VOLTAGE = 12.0;
   private static final double MIN_VALID_VOLTAGE = 8.0;
   private static final double MAX_VALID_VOLTAGE = 16.8;
   private static final double MIN_VOLTAGE_SCALE = 0.80;
   private static final double MAX_VOLTAGE_SCALE = 1.15;
   private static final double STICK_DEADBAND = 0.10;
   private static final double BATTERY_INTERVAL_S = 0.20;
   private static final double TELEMETRY_INTERVAL_S = 0.20;


   private Drivetrain2 drivetrain;
   private Odometry2 odometry;
   private VoltageSensor voltageSensor;
   private double batteryVoltage = Double.NaN; // actual reading; never clamp telemetry
   private double filteredVoltage = Double.NaN;
   private double voltageScale = 1.0;
   private boolean validVoltage;


   private final Gamepad currentGamepad = new Gamepad();
   private final Gamepad previousGamepad = new Gamepad();
   private final Gamepad driveGamepad = new Gamepad();
   private final ElapsedTime clock = new ElapsedTime();
   private double lastBatteryTime = -1.0;
   private double lastTelemetryTime = -1.0;
   private double lastLoopTime;
   private double smoothedLoopSeconds;


   @Override
   public void init() {
       for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
           hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
       }
       if (hardwareMap.voltageSensor.iterator().hasNext()) {
           voltageSensor = hardwareMap.voltageSensor.iterator().next();
       }
       updateBatteryVoltage(true);
       drivetrain = new Drivetrain2(hardwareMap);
       stopDrive();
       odometry = new Odometry2(hardwareMap, drivetrain);
       odometry.init();
       telemetry.setMsTransmissionInterval(200);
       sendTelemetry(true);
   }


   @Override
   public void init_loop() {
       updateBatteryVoltage(false);
       odometry.update(); // one Pinpoint update; getters use the cached sample
       odometry.updateReset();
       stopDrive();
       sendTelemetry(false);
   }


   @Override
   public void start() {
       lastLoopTime = clock.seconds();
       smoothedLoopSeconds = 0.0;
       currentGamepad.copy(gamepad1);
       previousGamepad.copy(currentGamepad);
       stopDrive();
   }


   @Override
   public void loop() {
       double now = clock.seconds();
       double dt = now - lastLoopTime;
       lastLoopTime = now;
       if (dt > 0.0) {
           smoothedLoopSeconds = smoothedLoopSeconds == 0.0 ? dt
                   : 0.9 * smoothedLoopSeconds + 0.1 * dt;
       }
       previousGamepad.copy(currentGamepad);
       currentGamepad.copy(gamepad1);
       updateBatteryVoltage(false);
       odometry.update();


       boolean driving = Math.abs(currentGamepad.left_stick_y) > STICK_DEADBAND
               || Math.abs(currentGamepad.left_stick_x) > STICK_DEADBAND
               || Math.abs(currentGamepad.right_stick_x) > STICK_DEADBAND;


       // During the settling stage a driver override can still cancel the reset.
       // An IMU calibration already sent to the device cannot be cancelled.
       if (driving) odometry.cancelPendingReset();
       if (odometry.isResetting()) {
           odometry.updateReset();
           stopDrive();
       } else if (!odometry.canDriveManually()) {
           stopDrive();
       } else if (driving) {
           odometry.cancelNavigation();
           driveGamepad.copy(currentGamepad);
           // Share the override deadband without assuming Drivetrain's shaping.
           if (Math.abs(driveGamepad.left_stick_y) <= STICK_DEADBAND)
               driveGamepad.left_stick_y = 0;
           if (Math.abs(driveGamepad.left_stick_x) <= STICK_DEADBAND)
               driveGamepad.left_stick_x = 0;
           if (Math.abs(driveGamepad.right_stick_x) <= STICK_DEADBAND)
               driveGamepad.right_stick_x = 0;
           drivetrain.update(driveGamepad, voltageScale);
       } else if (pressed(currentGamepad.back, previousGamepad.back)) {
           odometry.reset(); // exclusive priority: Back + Y cannot restart navigation
           stopDrive();
       } else if (pressed(currentGamepad.left_bumper, previousGamepad.left_bumper)) {
           odometry.syncTargetsToPose(); // cancels before changing the target
           stopDrive();
       } else {
           odometry.processInputs(
                   pressed(currentGamepad.dpad_up, previousGamepad.dpad_up),
                   pressed(currentGamepad.dpad_down, previousGamepad.dpad_down),
                   pressed(currentGamepad.dpad_left, previousGamepad.dpad_left),
                   pressed(currentGamepad.dpad_right, previousGamepad.dpad_right),
                   pressed(currentGamepad.x, previousGamepad.x),
                   pressed(currentGamepad.b, previousGamepad.b),
                   pressed(currentGamepad.y, previousGamepad.y));
           if (odometry.updateNavigation(voltageScale)) stopDrive();
       }
       sendTelemetry(false);
   }


   @Override
   public void stop() {
       try {
           if (odometry != null) odometry.cancelNavigation();
       } finally {
           stopDrive();
       }
   }


   private void stopDrive() {
       if (drivetrain != null) drivetrain.drive(0, 0, 0, 1.0);
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
       telemetry.addLine("PINEAPPLE2 -- METRIC");
       telemetry.addLine("D-pad: target 1 inch | X/B: angle left/right 5 deg");
       telemetry.addLine("Y: GO | sticks: manual/cancel | LB: cancel + sync target");
       telemetry.addData("Loop", "%.0f Hz / %.1f ms",
               smoothedLoopSeconds > 0.0 ? 1.0 / smoothedLoopSeconds : 0.0,
               smoothedLoopSeconds * 1000.0);
       telemetry.addData("Battery reading", "%.2f V", batteryVoltage);
       telemetry.addData("Battery compensation", validVoltage ? "VALID" : "INVALID / scale 1.0");
       telemetry.addData("Voltage scale", "%.3f", voltageScale);
       drivetrain.addTelemetry(telemetry);
       odometry.addTelemetry(telemetry);
       telemetry.update();
   }
}

