package org.firstinspires.ftc.teamcode;

/** Ball colour as classified by the colour sensors. */
public enum BallColor {
    WHITE, GREEN, PURPLE;

    @Override
    public String toString() {
        return name().charAt(0) + name().substring(3).toLowerCase();
    }
}