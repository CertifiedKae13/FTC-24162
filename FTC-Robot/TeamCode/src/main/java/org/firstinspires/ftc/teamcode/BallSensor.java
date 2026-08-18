package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Three colour sensors polled at a fixed interval; results are cached. */
public class BallSensor {
    private static final float HUE_PURPLE = 170f;
    private static final float HUE_GREEN = 100f;
    private static final long SENSOR_POLL_MS = 50;

    private final NormalizedColorSensor sensorLeft, sensorRight, sensorCenter;
    private final ElapsedTime sensorTimer = new ElapsedTime();

    private BallColor colorLeft = BallColor.WHITE;
    private BallColor colorRight = BallColor.WHITE;
    private BallColor colorCenter = BallColor.WHITE;

    public BallSensor(HardwareMap hardwareMap) {
        sensorLeft   = hardwareMap.get(NormalizedColorSensor.class, "color1");
        sensorRight  = hardwareMap.get(NormalizedColorSensor.class, "color2");
        sensorCenter = hardwareMap.get(NormalizedColorSensor.class, "color3");
    }

    public void update() {
        if (sensorTimer.milliseconds() >= SENSOR_POLL_MS) {
            colorLeft   = classify(hueOf(sensorLeft));
            colorRight  = classify(hueOf(sensorRight));
            colorCenter = classify(hueOf(sensorCenter));
            sensorTimer.reset();
        }
    }

    public BallColor left()   { return colorLeft; }
    public BallColor right()  { return colorRight; }
    public BallColor center() { return colorCenter; }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("── Balls ──");
        telemetry.addData(" Left", colorLeft);
        telemetry.addData(" Center", colorCenter);
        telemetry.addData(" Right", colorRight);
    }

    private float hueOf(NormalizedColorSensor s) {
        float[] hsv = new float[3];
        Color.colorToHSV(s.getNormalizedColors().toColor(), hsv);
        return hsv[0];
    }

    private BallColor classify(float hue) {
        if (hue > HUE_PURPLE) return BallColor.PURPLE;
        if (hue > HUE_GREEN) return BallColor.GREEN;
        return BallColor.WHITE;
    }
}
