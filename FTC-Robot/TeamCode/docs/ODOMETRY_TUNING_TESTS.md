# Individual odometry tuning tests for FTC-24162

This package adds Road Runner 1.0.x tuning to the existing FTC SDK 11.2.1 project. Its 16 entries appear under **Odometry Tuning** on the Driver Station. All 16 displayed test names now have Java entry-point files in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/tuning/`; they are registered by `TuningOpModes.java`. The five Road Runner push/ramp/feedforward entry points call the official library implementations so their recorded data works with the matching Dashboard graph pages. It leaves `MainTeleOP2`, `Drivetrain2`, and `Odometry2` intact. The tuning drive uses the robot's configured `frontLeft`, `frontRight`, `backLeft`, and `backRight` motors, its `imu`, and its `pinpoint` I2C device. The slides use `leftFront` and similar names; those are *not* the names used by this robot's current Java code.

## Before you run anything

1. Verify the Driver Station configuration has those exact case-sensitive names and the Pinpoint on the intended I2C port. Confirm that the parallel pod is connected to Pinpoint X and the perpendicular pod to Pinpoint Y. Wiring and app installation cannot be tested by Java alone.
2. Compile and install the Robot Controller app (`./gradlew :TeamCode:assembleDebug`). Connect to the RC Wi-Fi and open `http://192.168.43.1:8080/dash` for a Control Hub or `http://192.168.49.1:8080/dash` for a phone RC. Use the matching `/tuning/...` pages below for graph analysis.
3. Start with the robot lifted for the motor test, then put it on an open field for all moving tests. Keep a hand at STOP. Allow the robot to remain still during Pinpoint initialization. The ramp tests accelerate automatically; stop them before the robot reaches the field edge.
4. Record each result on paper or in source control. Dashboard edits are temporary; write confirmed values in the Java fields below and rebuild. The numbers shown in the slides are examples from another robot.

## Run one test at a time, in this order

| Slides | Driver Station OpMode | Action and expected result | Save/check |
| --- | --- | --- | --- |
| 1–5.4 | Hardware configuration, no OpMode | Check motor/pod wiring, RC app, Road Runner, Dashboard, case-sensitive names. `MecanumDrive` selects `PinpointLocalizer` and `TuningOpModes` selects mecanum. | `frontLeft`, `frontRight`, `backLeft`, `backRight`, `imu`, `pinpoint` must exist. |
| 5.2, 5.7 | `MecanumMotorDirectionDebugger` | Lift robot. Hold left bumper plus exactly one button: X=frontLeft, Y=frontRight, B=backRight, A=backLeft. Each selected wheel runs at 20% power while held and must spin physically forward. | If needed, change motor `setDirection(...)` in `MecanumDrive.java`; keep it consistent with `Drivetrain2.java`. |
| 5.5, 5.7 | `DeadWheelDirectionDebugger` | On field, push forward and confirm configured X position increases. Press A to reset displayed deltas, then push left and confirm configured Y position increases. Raw tick signs ignore Pinpoint encoder-direction settings. Motors stay off; the test also counts rejected I2C reads. | If needed, change `initialParDirection` or `initialPerpDirection` in `PinpointLocalizer.java`. This debugger's temporary direction changes are not saved. |
| 7.1–7.2 | `ForwardPushTest` | Slowly push forward a measured distance (about 60 in if room permits). Record positive ticks traveled. Repeat 3–5 times. | `MecanumDrive.PARAMS.inPerTick = measured inches / ticks`, averaged over runs. If ticks stay zero, check wiring and direction. |
| 8.1–8.4 | `ForwardRampLogger` | On open tiles, START, then STOP before perimeter. Analyze `/tuning/forward-ramp.html`; exclude genuine outliers. | Save graph's `kS` and `kV` to `MecanumDrive.PARAMS` in the labeled order. |
| 9.1–9.3 | `LateralRampLogger` | Leave open space on the robot's left. START and STOP before perimeter. Analyze `/tuning/lateral-ramp.html` with saved `inPerTick`, `kS`, `kV`. | Save `MecanumDrive.PARAMS.lateralInPerTick`. |
| 10.1–10.3 | `AngularRampLogger` | Clear turning space. Analyze `/tuning/dead-wheel-angular-ramp.html` using saved `kS`, `kV`. | Save `MecanumDrive.PARAMS.trackWidthTicks` and `PinpointLocalizer.PARAMS.parYTicks`, `perpXTicks`. |
| 11.1–11.3 | `ManualFeedforwardTuner` | Graph `vref` and `v0` in Dashboard. Adjust `kA` gradually; compare acceleration and braking. The library uses Y to pause/override and B to resume. | Save final `kS`, `kV`, `kA` in `MecanumDrive.PARAMS`. |
| 12.1–12.2 | `LocalizationTest` | Drive with gamepad at 35% speed; compare actual robot to Dashboard field drawing and displayed X/Y/heading. | Verify scale and axis signs before feedback. STOP halts drive. |
| 12.3 | `PinpointHeadingCheck` | With motors off, press A at a reference orientation. Rotate by hand 2–3 full turns. | CCW should read about +720° or +1080°; CW negative. Record error and Pinpoint status. |
| 12.4 | `PinpointForwardCheck` | Press A, push forward a known distance; motors remain off. | X forward must increase by measured inches. |
| 12.5 | `PinpointLateralCheck` | Press A, push left a known distance; motors remain off. | Y left must increase by measured inches. |
| 13.1–13.5 | `AxialFeedbackTest` | Set only `axialGain` above zero; run one 24-in forward/back cycle. Watch Dashboard target vs actual pose. | Tune `MecanumDrive.PARAMS.axialGain`; stop increasing if the robot oscillates. |
| 13.1–13.5 | `LateralFeedbackTest` | Set only `lateralGain` above zero; run one 24-in left/right cycle. | Tune `MecanumDrive.PARAMS.lateralGain`. |
| 13.1–13.5 | `HeadingFeedbackTest` | Set only `headingGain` above zero; run a 90° turn and return. | Tune `MecanumDrive.PARAMS.headingGain`. |
| 13.1–13.5 | `ManualFeedbackTuner` | Repeat 24-in forward/back cycles while comparing Dashboard target and actual. Stop if correction shakes. | Confirm the three gains together. Velocity gains may stay zero. |
| 14 | `SplineTest` | One spline from (0,0) to (18,12) to (0,24) in. Make sure that space is clear. | Position and heading should follow the path without large drift. |

The trajectory and feedback tests refuse to run until the main calibration constants have been saved in Java. The initial profile is limited to 24 in/s and 90°/s, but the robot still needs open floor and someone ready to STOP. The `PinpointManualCheck` modes never command motor power.

## If `DeadWheelDirectionDebugger` shows `FAULT_BAD_READ`

The FTC SDK reports this when it rejects an I2C read and retains the last good data. A single rejected read can recover; the test shows good/bad read counts and stops treating stale data as a new measurement. If it reports 20 consecutive bad reads (about two seconds), stop tuning and check the Pinpoint's power, I2C cable and connector, hub port, and Driver Station configuration (`pinpoint`, goBILDA Pinpoint Odometry Computer). Check the physical status LED. A green LED with persistent bad reads points toward the I2C connection or configuration; purple/blue/orange indicates a pod detection issue. Do not change encoder directions or calibration values to hide an I2C read fault.

This debugger no longer resets Pinpoint's IMU on INIT and reads at 10 Hz. The motor test can run without Pinpoint, so passing it does not prove the I2C connection works.

## Important slide corrections

- In the official Road Runner `PinpointLocalizer`, `parYTicks` and `perpXTicks` are **pod location offsets**, measured in ticks and converted to millimeters for Pinpoint's `setOffsets`. They are not encoder ticks per revolution. Do not adjust them merely because straight-line travel has the wrong scale. First verify `inPerTick`/encoder resolution and wheel contact. Use the offsets when rotation introduces false X/Y translation.
- If heading is wrong after multiple full turns, verify Pinpoint is firmly mounted, initialized while still, and reports READY. Moving it closer to the robot center is not a general heading-scale fix. Check the manufacturer's yaw calibration guidance before changing yaw scalar.
- `lateralInPerTick` models mecanum drive strafing in Road Runner. It does not change Pinpoint's Y distance scale.
- Road Runner tuning values do **not** automatically retune `MainTeleOP2`: that OpMode uses `Drivetrain2` and `Odometry2`. After validating this suite, transfer the measured Pinpoint resolution and physical offsets deliberately. The Road Runner offset conversion is `offset_mm = inPerTick * 25.4 * offset_ticks`. Road Runner's feedback gains use different units and should not be pasted into `Odometry2` without retuning its controller.

## Code locations and cleanup

- `MecanumDrive.java`: motor names and directions, localizer choice, `inPerTick`, `lateralInPerTick`, `trackWidthTicks`, `kS`, `kV`, `kA`, and feedback gains.
- `PinpointLocalizer.java`: parallel/perpendicular encoder directions and pod offsets.
- `tuning/TuningOpModes.java`: individual Driver Station registration. Change `DISABLED` to `true` after tuning to hide the suite.
- `tuning/ForwardPushTest.java`, `ForwardRampLogger.java`, `LateralRampLogger.java`, `AngularRampLogger.java`, `ManualFeedforwardTuner.java`: local Java entry points that construct the official Road Runner implementations. Their measurement algorithms remain in the `com.acmerobotics.roadrunner:ftc` dependency; the local files show exactly which library test runs.
- `tuning/LocalizationTest.java`: slow gamepad drive and Dashboard field pose.

These files adapt the official Road Runner quickstart. Its repository license is retained by the FTC project, and the Road Runner library is installed as a dependency. Keep the robot's existing application code when updating the tuning suite.

## References

- [Road Runner 1.0.x installation](https://rr.brott.dev/docs/v1-0/installation/)
- [Road Runner 1.0.x tuning](https://rr.brott.dev/docs/v1-0/tuning)
- [Official Road Runner quickstart](https://github.com/acmerobotics/road-runner-quickstart)
- [goBILDA Pinpoint user guide](https://www.gobilda.com/content/user_manuals/3110-0002-0001_user-guide.pdf)
