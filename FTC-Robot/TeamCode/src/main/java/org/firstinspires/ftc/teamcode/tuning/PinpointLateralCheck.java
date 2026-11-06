package org.firstinspires.ftc.teamcode.tuning;

/** Slide 12.5: measure a hand-pushed left/right run. */
public final class PinpointLateralCheck extends PinpointManualCheck {
    @Override protected String testName() { return "Leftward distance"; }
    @Override protected String instruction() {
        return "Push left a measured distance; Y should rise by that many inches.";
    }
    @Override protected double measurement(double dx, double dy, double dh) { return dy; }
    @Override protected String unit() { return "in"; }
}
