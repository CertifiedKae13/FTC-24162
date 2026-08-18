package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * ─── CONTROLS ────────────────────────────────────────────────
 * Left stick         Mecanum drive (forward / strafe)
 * Right stick        Chassis turn (X) — turret feed-forward compensates
 * L/R trigger        Manual turret rotation
 * A                  START launcher wheel (velocity set by D‑pad)
 * B                  STOP launcher wheel
 * B + R-bumper       Reverse intake
 * X                  Sort PURPLE ball to center (delayed for putter)
 * Y                  Sort GREEN ball to center (delayed for putter)
 * L-bumper           Rotate plate CW (immediate)
 * R-bumper           Rotate plate CCW (immediate)
 * D-pad up           Launcher EXTENDED (0.6) + speed -2500
 * D-pad down         Launcher RETRACTED (0.9) + speed -1500
 * D-pad left         Toggle intake
 * D-pad right        Shake plate
 * L/R stick click    Fire putter
 * Back               Toggle auto-tracking
 * ─────────────────────────────────────────────────────────────
 */

@TeleOp(name = "MaOp v2")
public class LimelightTest extends OpMode {

    // ═══════════════════════════════════════════════════════
    // TUNING CONSTANTS
    // ═══════════════════════════════════════════════════════
    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double MIN_VOLTAGE = 8.0;

    // ═══════════════════════════════════════════════════════
    // SUBSYSTEMS
    // ═══════════════════════════════════════════════════════
    private Drivetrain drivetrain;
    private Intake intake;
    private Putter putter;
    private Launcher launcher;
    private Plate plate;
    private Turret turret;
    private BallSensor ballSensor;

    // ═══════════════════════════════════════════════════════
    // STATE
    // ═══════════════════════════════════════════════════════
    private VoltageSensor voltageSensor;
    private double batteryVoltage = 12.0;

    private final Gamepad currGP = new Gamepad();
    private final Gamepad prevGP = new Gamepad();
    private final ElapsedTime loopTimer = new ElapsedTime();

    // ═══════════════════════════════════════════════════════
    // INIT
    // ═══════════════════════════════════════════════════════
    @Override
    public void init() {
        enableBulkReads();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        drivetrain = new Drivetrain(hardwareMap);
        intake     = new Intake(hardwareMap);
        putter     = new Putter(hardwareMap);
        launcher   = new Launcher(hardwareMap);
        ballSensor = new BallSensor(hardwareMap);
        plate      = new Plate(hardwareMap, ballSensor, putter);
        turret     = new Turret(hardwareMap);

        telemetry.addLine("✓ All systems initialized");
        telemetry.update();
    }

    private void enableBulkReads() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    // ═══════════════════════════════════════════════════════
    // START · LOOP · STOP
    // ═══════════════════════════════════════════════════════
    @Override
    public void start() {
        turret.start();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        snapshotGamepad();
        batteryVoltage = Math.max(voltageSensor.getVoltage(), MIN_VOLTAGE);
        double voltageScale = NOMINAL_VOLTAGE / batteryVoltage;

        drivetrain.update(currGP, voltageScale);
        intake.update(currGP, prevGP, voltageScale);
        putter.update(currGP, prevGP);
        launcher.update(currGP, prevGP);
        plate.update(currGP, prevGP);
        turret.update(currGP, prevGP, dt);
        ballSensor.update();

        emitTelemetry(dt);
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
    }

    // ═══════════════════════════════════════════════════════
    // HELPERS
    // ═══════════════════════════════════════════════════════
    private void snapshotGamepad() {
        prevGP.copy(currGP);
        currGP.copy(gamepad1);
    }

    private void emitTelemetry(double dt) {
        telemetry.addData("Loop", "%.0f Hz", 1.0 / Math.max(dt, 1e-6));
        telemetry.addData("Battery", "%.1fV (×%.2f)", batteryVoltage, NOMINAL_VOLTAGE / batteryVoltage);

        turret.addTelemetry(telemetry);
        ballSensor.addTelemetry(telemetry);
        drivetrain.addTelemetry(telemetry);
        plate.addTelemetry(telemetry);
        launcher.addTelemetry(telemetry);
    }
}
