package org.firstinspires.ftc.teamcode.tuning;

/** Slide 12.4: measure a hand-pushed forward/backward run. */
public final class PinpointForwardCheck extends PinpointManualCheck {
    @Override protected String testName() { return "Forward distance"; }
    @Override protected String instruction() {
        return "Push forward a measured distance; X should rise by that many inches.";
    }
    @Override protected double measurement(double dx, double dy, double dh) { return dx; }
    @Override protected String unit() { return "in"; }
}
