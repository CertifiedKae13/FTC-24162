package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Runs the Road Runner forward ramp and writes data for forward-ramp.html.
 * This Java entry point keeps the official Road Runner data format used by its
 * Dashboard tuning page. The measurement algorithm is in the Road Runner FTC
 * dependency declared in TeamCode/build.gradle.
 */
public final class ForwardRampLogger {
    private ForwardRampLogger() { }

    public static OpMode create(DriveViewFactory factory) {
        return new com.acmerobotics.roadrunner.ftc.ForwardRampLogger(factory);
    }
}
