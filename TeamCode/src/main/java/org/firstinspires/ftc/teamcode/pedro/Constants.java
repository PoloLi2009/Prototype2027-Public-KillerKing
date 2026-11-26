package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.OptionalDouble;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.PinpointConstants;

/** Pedro Pathing configuration for Killer King. Hardware names come from the TeleOp constants. */
public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(DriveConstants.FRONT_LEFT_MOTOR_NAME);
        c.backLeftName.set(DriveConstants.BACK_LEFT_MOTOR_NAME);
        c.frontRightName.set(DriveConstants.FRONT_RIGHT_MOTOR_NAME);
        c.backRightName.set(DriveConstants.BACK_RIGHT_MOTOR_NAME);
        // Same directions as the TeleOp Drive subsystem (left FORWARD, right REVERSE).
        c.frontLeftDirection.set(DriveConstants.LEFT_MOTOR_DIRECTION);
        c.backLeftDirection.set(DriveConstants.LEFT_MOTOR_DIRECTION);
        c.frontRightDirection.set(DriveConstants.RIGHT_MOTOR_DIRECTION);
        c.backRightDirection.set(DriveConstants.RIGHT_MOTOR_DIRECTION);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(PinpointConstants.DEVICE_NAME);
        c.xPodOffset.set(PinpointConstants.X_OFFSET_MM);
        c.yPodOffset.set(PinpointConstants.Y_OFFSET_MM);
        c.offsetUnits.set(DistanceUnit.MM);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        // AutoTune Pinpoint Tuner (2026-11-26) confirmed 4-bar pods and both directions FORWARD. Its
        // spin-based offsets (-68.5, -143.1 mm) were not used: the route assumes the footprint center,
        // so the measured TeleOp offsets stay.
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodDirection.set(PinpointConstants.X_ENCODER_DIRECTION);
        c.yPodDirection.set(PinpointConstants.Y_ENCODER_DIRECTION);
    });

    /**
     * AutoTune Foresight export, second run (2026-11-26), with DriveConstants.frontStrafeScale = 1.0.
     * Max strafe velocity is 58.5 in/s (70% of forward), against 21.8 in/s in the first run while
     * the chassis still yawed about 30 deg when strafing. Re-run Foresight and replace this whole
     * block after any drivetrain change or a new frontStrafeScale. Autonomous refuses to drive if
     * this is null.
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.29051173769663);
                Controller secondaryTranslationalForward = Controller.proportional(0.10733632446230737);
                Controller primaryTranslationalLateral = Controller.proportional(0.35773613725766984);
                Controller secondaryTranslationalLateral = Controller.proportional(0.13217394383107298);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010647373443948888));
                c.brake.set(Controller.proportionalFeedforward(0.009050267427356555));

                c.headingFeedback.set(Controller.proportional(3.7896694900028614));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.045615092210871996, 0.006879128631569607));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10006900983091568, 0.02828458754860449));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.00186154681863538, 0.0031064289789791916));

                c.maxAchievableForwardVelocity.set(83.70364834637866);
                c.maxAchievableStrafeVelocity.set(58.46072015841529);
                c.naturalForwardDeceleration.set(25.462396361414296);
                c.naturalStrafeDeceleration.set(51.95018804395945);
            }
    );

    public static Foresight createAlgorithm() {
        if (foresightConfig == null) {
            throw new IllegalStateException(
                    "Run AutoTune > Foresight Tuner, then paste its Java export into pedro/Constants.java");
        }
        return new Foresight(foresightConfig);
    }

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new CompensatedMecanum(hardwareMap, drivetrainConfig),   // strafe yaw compensation
                createAlgorithm());
    }
}
