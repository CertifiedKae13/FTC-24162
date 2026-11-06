package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;

/** Gate trajectory tests until the preceding slide measurements are saved. */
final class TuningSafety {
    static final PoseVelocity2d ZERO_POWER = new PoseVelocity2d(new Vector2d(0, 0), 0);
    private TuningSafety() { }
    static boolean calibrated() {
        return MecanumDrive.PARAMS.inPerTick > 0
                && MecanumDrive.PARAMS.inPerTick != 1.0
                && MecanumDrive.PARAMS.lateralInPerTick > 0
                && MecanumDrive.PARAMS.lateralInPerTick != 1.0
                && MecanumDrive.PARAMS.trackWidthTicks > 0
                && MecanumDrive.PARAMS.kV > 0
                && (PinpointLocalizer.PARAMS.parYTicks != 0
                || PinpointLocalizer.PARAMS.perpXTicks != 0);
    }
    static String missing() {
        return "Need inPerTick, lateralInPerTick, trackWidthTicks, kV, "
                + "parYTicks and perpXTicks saved in Java.";
    }
}
