package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Reads raw forward encoder ticks while the robot is pushed by hand.
 * This Java entry point keeps the official Road Runner data format used by its
 * Dashboard tuning page. The measurement algorithm is in the Road Runner FTC
 * dependency declared in TeamCode/build.gradle.
 */
public final class ForwardPushTest {
    private ForwardPushTest() { }

    public static OpMode create(DriveViewFactory factory) {
        return new com.acmerobotics.roadrunner.ftc.ForwardPushTest(factory);
    }
}
