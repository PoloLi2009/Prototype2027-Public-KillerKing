package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;

public final class DriveConstants {
    private DriveConstants() { }

    public static final String FRONT_LEFT_MOTOR_NAME = "0";
    public static final String BACK_LEFT_MOTOR_NAME = "1";
    public static final String FRONT_RIGHT_MOTOR_NAME = "2";
    public static final String BACK_RIGHT_MOTOR_NAME = "3";
    public static final String IMU_NAME = "imu";

    public static final DcMotor.RunMode MOTOR_RUN_MODE = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;
    public static final DcMotor.Direction MOTOR_DIRECTION = DcMotor.Direction.REVERSE;
    public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    public static final double JOYSTICK_DEADZONE = 0.05;
}
