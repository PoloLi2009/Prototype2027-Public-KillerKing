package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.PinpointConstants;

/** Pedro Pathing configuration for Killer King. Hardware names come from the TeleOp constants. */
public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(DriveConstants.FRONT_LEFT_MOTOR_NAME);
        c.backLeftName.set(DriveConstants.BACK_LEFT_MOTOR_NAME);
        c.frontRightName.set(DriveConstants.FRONT_RIGHT_MOTOR_NAME);
        c.backRightName.set(DriveConstants.BACK_RIGHT_MOTOR_NAME);
        // Killer King runs all four drive motors REVERSE (see DriveConstants / README).
        c.frontLeftDirection.set(DriveConstants.MOTOR_DIRECTION);
        c.backLeftDirection.set(DriveConstants.MOTOR_DIRECTION);
        c.frontRightDirection.set(DriveConstants.MOTOR_DIRECTION);
        c.backRightDirection.set(DriveConstants.MOTOR_DIRECTION);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(PinpointConstants.DEVICE_NAME);
        c.xPodOffset.set(PinpointConstants.X_OFFSET_MM);
        c.yPodOffset.set(PinpointConstants.Y_OFFSET_MM);
        c.offsetUnits.set(DistanceUnit.MM);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        // TODO confirm: the TeleOp never sets a pod type, so this assumes goBILDA 4-bar pods.
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodDirection.set(PinpointConstants.X_ENCODER_DIRECTION);
        c.yPodDirection.set(PinpointConstants.Y_ENCODER_DIRECTION);
    });

    /**
     * Killer King has not been tuned yet. Run AutoTune > Mecanum, Pinpoint, then Foresight on the
     * robot and paste the Foresight Java export here. Autonomous refuses to drive while this is null.
     */
    public static ForesightConfig foresightConfig = null;

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
                new Mecanum(hardwareMap, drivetrainConfig),
                createAlgorithm());
    }
}
