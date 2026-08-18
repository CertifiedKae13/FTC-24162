package org.firstinspires.ftc.teamcode;

/** Launcher servo angle. */
public enum LaunchAngle {
    RETRACTED(0.90),
    EXTENDED(0.50);

    final double servoPos;

    LaunchAngle(double servoPos) { this.servoPos = servoPos; }
}
