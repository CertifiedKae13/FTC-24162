package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Rotating plate: 3 slots, manual rotation, colour sort, and shake. */
public class Plate {
    private static final long PLATE_DELAY_MS = 650;
    private static final double[] PLATE_POSITIONS = { 0.07, 0.5, 0.95 };
    private static final double SHAKE_AMPLITUDE = 0.20;
    private static final long SHAKE_HALF_MS = 200;

    private final Servo plateServo;
    private final BallSensor ballSensor;
    private final Putter putter;

    private final ElapsedTime plateDelayTimer = new ElapsedTime();
    private final ElapsedTime shakeTimer = new ElapsedTime();

    private int plateIndex = 0;
    private boolean rotationPending = false;
    private int pendingPlateIndex = 0;
    private boolean shaking = false;

    public Plate(HardwareMap hardwareMap, BallSensor ballSensor, Putter putter) {
        this.ballSensor = ballSensor;
        this.putter = putter;
        plateServo = hardwareMap.get(Servo.class, "plateServo");
        plateServo.setPosition(PLATE_POSITIONS[0]);
    }

    public void update(Gamepad curr, Gamepad prev) {
        BallColor colorCenter = ballSensor.center();

        // ── Sort to centre (X = purple, Y = green), delayed for the putter ──
        if (GamepadUtil.rising(curr.x, prev.x) && colorCenter != BallColor.PURPLE) {
            int idx = sortTarget(BallColor.PURPLE);
            if (idx != plateIndex) queueRotation(idx);
        } else if (GamepadUtil.rising(curr.y, prev.y) && colorCenter != BallColor.GREEN) {
            int idx = sortTarget(BallColor.GREEN);
            if (idx != plateIndex) queueRotation(idx);
        }

        // ── Manual rotation (bumpers) ──
        if (GamepadUtil.rising(curr.right_bumper, prev.right_bumper) && !curr.b) {
            plateIndex = Math.floorMod(plateIndex - 1, 3);
            rotationPending = false;
        } else if (GamepadUtil.rising(curr.left_bumper, prev.left_bumper)) {
            plateIndex = Math.floorMod(plateIndex + 1, 3);
            rotationPending = false;
        }

        // ── Resolve a pending rotation once the delay passes and the putter is clear ──
        if (rotationPending) {
            boolean delayMet = plateDelayTimer.milliseconds() >= PLATE_DELAY_MS;
            boolean putterSafe = putter.isClear();
            if (delayMet && putterSafe) {
                plateIndex = pendingPlateIndex;
                rotationPending = false;
            }
        }

        // ── Shake (D-pad right) ──
        if (GamepadUtil.rising(curr.dpad_right, prev.dpad_right) && !shaking) {
            shaking = true;
            shakeTimer.reset();
        }

        if (shaking) {
            double base = platePosition();
            long elapsed = (long) shakeTimer.milliseconds();
            if (elapsed < SHAKE_HALF_MS) {
                plateServo.setPosition(Range.clip(base - SHAKE_AMPLITUDE, 0, 1));
            } else if (elapsed < SHAKE_HALF_MS * 2) {
                plateServo.setPosition(Range.clip(base + SHAKE_AMPLITUDE, 0, 1));
            } else {
                plateServo.setPosition(base);
                shaking = false;
            }
        } else {
            plateServo.setPosition(platePosition());
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("── Plate ──");
        telemetry.addData(" Slot", plateIndex);
        telemetry.addData(" Servo", "%.3f", platePosition());
        telemetry.addData(" Pending", rotationPending ? "WAITING" : "—");
    }

    private void queueRotation(int targetIndex) {
        pendingPlateIndex = targetIndex;
        rotationPending = true;
        plateDelayTimer.reset();
    }

    private int sortTarget(BallColor target) {
        if (ballSensor.left() == target) return Math.floorMod(plateIndex + 1, 3);
        else if (ballSensor.right() == target) return Math.floorMod(plateIndex - 1, 3);
        return plateIndex;
    }

    private double platePosition() {
        return PLATE_POSITIONS[plateIndex];
    }
}
