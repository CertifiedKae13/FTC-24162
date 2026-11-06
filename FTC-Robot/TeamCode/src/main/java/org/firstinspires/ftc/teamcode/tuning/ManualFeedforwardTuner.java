package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Runs the Road Runner forward/back feedforward tuner and Dashboard telemetry.
 * This Java entry point keeps the official Road Runner data format used by its
 * Dashboard tuning page. The measurement algorithm is in the Road Runner FTC
 * dependency declared in TeamCode/build.gradle.
 */
public final class ManualFeedforwardTuner {
    private ManualFeedforwardTuner() { }

    public static OpMode create(DriveViewFactory factory) {
        return new com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner(factory);
    }
}
