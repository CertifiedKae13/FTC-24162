package org.firstinspires.ftc.teamcode.tuning;

/** Slide 12.3: rotate by hand 2-3 full turns and inspect heading drift. */
public final class PinpointHeadingCheck extends PinpointManualCheck {
    @Override protected String testName() { return "CCW rotation"; }
    @Override protected String instruction() {
        return "Rotate CCW 2 or 3 turns by hand; expect about +720 or +1080 degrees.";
    }
    @Override protected double measurement(double dx, double dy, double dh) { return dh; }
    @Override protected String unit() { return "deg"; }
}
