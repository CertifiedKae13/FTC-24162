package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Replays the newest CaptureLog file on the FTC Dashboard field view, in the original timing. Needs no robot
 * hardware, so you can run it with the robot parked. Press A to play it again.
 */
@TeleOp(name = "Replay Last Auto", group = "Test")
public class ReplayLastAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        List<double[]> rows = new ArrayList<>();
        String name = loadNewest(rows);
        telemetry.addData("Capture", name == null ? "none found in " + CaptureLog.folder() : name + " (" + rows.size() + " rows)");
        telemetry.addLine("Open the Dashboard field view, press start; A replays again");
        telemetry.update();
        waitForStart();
        boolean again = true;
        while (opModeIsActive()) {
            if (again && !rows.isEmpty()) {
                long start = System.currentTimeMillis();
                for (double[] r : rows) {
                    while (opModeIsActive() && System.currentTimeMillis() - start < r[0]) sleep(5);
                    if (!opModeIsActive()) return;
                    PoseDisplay.setExtras(r[8], null);
                    PoseDisplay.show(telemetry, r[1], r[2], r[3], r[4], r[5], r[6], r[7]);
                    telemetry.addData("Replay", "%.1f s", r[0] / 1000.0);
                    telemetry.update();
                }
                telemetry.addData("Replay", "done, press A to play again");
                telemetry.update();
            }
            again = gamepad1.a;
            sleep(20);
        }
    }

    private static String loadNewest(List<double[]> rows) {
        File[] files = CaptureLog.folder().listFiles((d, n) -> n.endsWith(".csv"));
        if (files == null || files.length == 0) return null;
        File newest = files[0];
        for (File f : files) if (f.getName().compareTo(newest.getName()) > 0) newest = f;
        try (BufferedReader br = new BufferedReader(new FileReader(newest))) {
            String line = br.readLine();                       // header
            while ((line = br.readLine()) != null) {
                String[] p = line.split(",");
                if (p.length < 9) continue;
                double[] v = new double[9];
                for (int i = 0; i < 9; i++) v[i] = Double.parseDouble(p[i]);
                rows.add(v);
            }
        } catch (IOException | NumberFormatException e) {
            return newest.getName() + " (unreadable: " + e.getMessage() + ")";
        }
        return newest.getName();
    }
}
