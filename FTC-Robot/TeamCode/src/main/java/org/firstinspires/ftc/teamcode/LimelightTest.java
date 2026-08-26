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
 * Right stick        Chassis turn (X)
 * ─────────────────────────────────────────────────────────────
 */

@TeleOp(name = "oPoP")
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

        if (hardwareMap.voltageSensor.iterator().hasNext()) {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        }

        drivetrain = new Drivetrain(hardwareMap);

        telemetry.addLine("✓ Drivetrain initialized");
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
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        snapshotGamepad();

        if (voltageSensor != null) {
            batteryVoltage = Math.max(
                    voltageSensor.getVoltage(),
                    MIN_VOLTAGE
            );
        } else {
            batteryVoltage = NOMINAL_VOLTAGE;
        }

        double voltageScale =
                NOMINAL_VOLTAGE / batteryVoltage;

        drivetrain.update(currGP, voltageScale);

        emitTelemetry(dt);
        telemetry.update();
    }

    @Override
    public void stop() {
        // No subsystem shutdown required
    }

    // ═══════════════════════════════════════════════════════
    // HELPERS
    // ═══════════════════════════════════════════════════════
    private void snapshotGamepad() {
        prevGP.copy(currGP);
        currGP.copy(gamepad1);
    }

    private void emitTelemetry(double dt) {
        telemetry.addData(
                "Loop",
                "%.0f Hz",
                1.0 / Math.max(dt, 1e-6)
        );

        telemetry.addData(
                "Battery",
                "%.1fV (×%.2f)",
                batteryVoltage,
                NOMINAL_VOLTAGE / batteryVoltage
        );

        drivetrain.addTelemetry(telemetry);
    }
}