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
     * AutoTune Foresight export, 2026-11-26, with DriveConstants.frontStrafeScale = 1.0 (no strafe
     * compensation) while the chassis still yawed about 30 deg when strafing. The forward and heading
     * values look sound; the strafe ones (max strafe velocity, strafe deceleration and braking, strafe
     * translational) were measured with that yaw. Re-run Foresight after the strafe is fixed or the
     * compensation is set, and replace this whole block. Autonomous refuses to drive if this is null.
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3146650788727505);
                Controller secondaryTranslationalForward = Controller.proportional(0.11626033863772138);
                Controller primaryTranslationalLateral = Controller.proportional(0.3991613107515434);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1474794385360374);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011608935753202404));
                c.brake.set(Controller.proportionalFeedforward(0.009867595390222044));

                c.headingFeedback.set(Controller.proportional(4.253358625038861));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.051775604289299774, 0.006298193897146152));

                c.linearBrakeCoefficients.set(Matrix.diag(0.12442525105231204, 0.008197113422656303));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015138867118045039, 0.0031732933170629894));

                c.maxAchievableForwardVelocity.set(74.96601576339545);
                c.maxAchievableStrafeVelocity.set(21.814159345924665);
                c.naturalForwardDeceleration.set(26.316156290549884);
                c.naturalStrafeDeceleration.set(55.451491682113506);
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
