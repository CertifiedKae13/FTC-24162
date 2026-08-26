package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Mecanum drivetrain with voltage compensation. */
public class Drivetrain {
    private static final double STRAFE_SCALE = 1.1;

    private final DcMotor frontLeft, frontRight, backLeft, backRight;

    // Cached stick values (kept for telemetry).
    private double fwd, strafe, turn;

    public Drivetrain(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void update(Gamepad gamepad, double voltageScale) {
        fwd    = -cube(gamepad.left_stick_y);
        strafe = cube(gamepad.left_stick_x) * STRAFE_SCALE;
        turn   = cube(gamepad.right_stick_x);

        double denom = Math.max(Math.abs(fwd) + Math.abs(strafe) + Math.abs(turn), 1.0);

        frontLeft.setPower(Range.clip(((fwd + strafe + turn) / denom) * voltageScale, -1.0, 1.0));
        backLeft.setPower(Range.clip(((fwd - strafe + turn) / denom) * voltageScale, -1.0, 1.0));
        frontRight.setPower(Range.clip(((fwd - strafe - turn) / denom) * voltageScale, -1.0, 1.0));
        backRight.setPower(Range.clip(((fwd + strafe - turn) / denom) * voltageScale, -1.0, 1.0));
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Update success", 0);
        telemetry.addData("FR Power:", frontRight.getPower());
        telemetry.addData("FL Power:", frontLeft.getPower());
        telemetry.addData("BR Power:", backRight.getPower());
        telemetry.addData("BL Power:", backLeft.getPower());
        telemetry.addData("FWD:", fwd);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("turn", turn);
    }

    private static double cube(double v) {
        return v * v * v;
    }
}
