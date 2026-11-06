package org.firstinspires.ftc.teamcode.tuning;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.EncoderGroup;
import com.acmerobotics.roadrunner.ftc.EncoderRef;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.PinpointEncoderGroup;
import com.acmerobotics.roadrunner.ftc.PinpointIMU;
import com.acmerobotics.roadrunner.ftc.PinpointView;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointLocalizer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Road Runner tuning registration for this robot's mecanum + Pinpoint hardware. */
public final class TuningOpModes {
    public static final Class<?> DRIVE_CLASS = MecanumDrive.class;
    public static final String GROUP = "Odometry Tuning";
    public static final boolean DISABLED = true;

    private TuningOpModes() { }

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return meta(cls.getSimpleName());
    }

    private static OpModeMeta meta(String name) {
        return new OpModeMeta.Builder().setName(name)
                .setGroup(GROUP).setFlavor(OpModeMeta.Flavor.TELEOP).build();
    }

    private static PinpointView pinpointView(PinpointLocalizer localizer) {
        return new PinpointView() {
            GoBildaPinpointDriver.EncoderDirection parDirection = localizer.initialParDirection;
            GoBildaPinpointDriver.EncoderDirection perpDirection = localizer.initialPerpDirection;

            @Override public void update() { localizer.driver.update(); }
            @Override public int getParEncoderPosition() { return localizer.driver.getEncoderX(); }
            @Override public int getPerpEncoderPosition() { return localizer.driver.getEncoderY(); }
            @Override public float getHeadingVelocity(UnnormalizedAngleUnit unit) {
                return (float) localizer.driver.getHeadingVelocity(unit);
            }
            @Override public void setParDirection(@NonNull DcMotorSimple.Direction direction) {
                parDirection = direction == DcMotorSimple.Direction.FORWARD
                        ? GoBildaPinpointDriver.EncoderDirection.FORWARD
                        : GoBildaPinpointDriver.EncoderDirection.REVERSED;
                localizer.driver.setEncoderDirections(parDirection, perpDirection);
            }
            @Override public DcMotorSimple.Direction getParDirection() {
                return parDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD
                        ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
            }
            @Override public void setPerpDirection(@NonNull DcMotorSimple.Direction direction) {
                perpDirection = direction == DcMotorSimple.Direction.FORWARD
                        ? GoBildaPinpointDriver.EncoderDirection.FORWARD
                        : GoBildaPinpointDriver.EncoderDirection.REVERSED;
                localizer.driver.setEncoderDirections(parDirection, perpDirection);
            }
            @Override public DcMotorSimple.Direction getPerpDirection() {
                return perpDirection == GoBildaPinpointDriver.EncoderDirection.FORWARD
                        ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
            }
        };
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        DriveViewFactory factory = hardwareMap -> {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            PinpointView pinpoint = pinpointView((PinpointLocalizer) drive.localizer);
            List<EncoderGroup> groups = new ArrayList<>();
            groups.add(new PinpointEncoderGroup(pinpoint));
            List<EncoderRef> parallel = Arrays.asList(new EncoderRef(0, 0));
            List<EncoderRef> perpendicular = Arrays.asList(new EncoderRef(0, 1));
            LazyImu imu = new PinpointIMU(pinpoint);
            return new DriveView(
                    DriveType.MECANUM,
                    MecanumDrive.PARAMS.inPerTick,
                    MecanumDrive.PARAMS.maxWheelVel,
                    MecanumDrive.PARAMS.minProfileAccel,
                    MecanumDrive.PARAMS.maxProfileAccel,
                    groups,
                    Arrays.asList(drive.leftFront, drive.leftBack),
                    Arrays.asList(drive.rightFront, drive.rightBack),
                    new ArrayList<>(), new ArrayList<>(), parallel, perpendicular,
                    imu, drive.voltageSensor,
                    () -> new MotorFeedforward(MecanumDrive.PARAMS.kS,
                            MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                            MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick),
                    0
            );
        };

        manager.register(metaForClass(MecanumMotorDirectionDebugger.class),
                MecanumMotorDirectionDebugger.class);
        manager.register(metaForClass(DeadWheelDirectionDebugger.class),
                DeadWheelDirectionDebugger.class);
        manager.register(meta("ForwardPushTest"), ForwardPushTest.create(factory));
        manager.register(meta("ForwardRampLogger"), ForwardRampLogger.create(factory));
        manager.register(meta("LateralRampLogger"), LateralRampLogger.create(factory));
        manager.register(meta("AngularRampLogger"), AngularRampLogger.create(factory));
        manager.register(meta("ManualFeedforwardTuner"),
                ManualFeedforwardTuner.create(factory));
        manager.register(metaForClass(ManualFeedbackTuner.class), ManualFeedbackTuner.class);
        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);
        manager.register(metaForClass(PinpointForwardCheck.class), PinpointForwardCheck.class);
        manager.register(metaForClass(PinpointLateralCheck.class), PinpointLateralCheck.class);
        manager.register(metaForClass(PinpointHeadingCheck.class), PinpointHeadingCheck.class);
        manager.register(metaForClass(AxialFeedbackTest.class), AxialFeedbackTest.class);
        manager.register(metaForClass(LateralFeedbackTest.class), LateralFeedbackTest.class);
        manager.register(metaForClass(HeadingFeedbackTest.class), HeadingFeedbackTest.class);
        manager.register(metaForClass(SplineTest.class), SplineTest.class);

        FtcDashboard.getInstance().withConfigRoot(root -> {
            for (Class<?> cls : Arrays.asList(
                    com.acmerobotics.roadrunner.ftc.AngularRampLogger.class,
                    com.acmerobotics.roadrunner.ftc.ForwardRampLogger.class,
                    com.acmerobotics.roadrunner.ftc.LateralRampLogger.class,
                    com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner.class,
                    ManualFeedbackTuner.class)) {
                root.putVariable(cls.getSimpleName(), ReflectionConfig.createVariableFromClass(cls));
            }
        });
    }
}
