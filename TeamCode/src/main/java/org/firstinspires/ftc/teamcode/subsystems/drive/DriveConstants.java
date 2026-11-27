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
    public static final DcMotor.Direction LEFT_MOTOR_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction RIGHT_MOTOR_DIRECTION = DcMotor.Direction.REVERSE;
    public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

    public static final double JOYSTICK_DEADZONE = 0.05;

    // TeleOp driver forward at START as a field heading: away from the own alliance wall (red +X,
    // blue -X). The TeleOp subtracts the Pinpoint start heading to get it relative to the robot, so
    // the robot must start at that heading. (With the old 180 deg start this gave red 180 = robot
    // rear and blue 0 = robot front.)
    public static final double RED_DRIVER_FORWARD_FIELD_DEGREES = 0.0;
    public static final double BLUE_DRIVER_FORWARD_FIELD_DEGREES = 180.0;

    // Strafe yaw compensation, shared by the TeleOp Drive and Pedro (CompensatedMecanum). The front
    // wheels' share of every strafe command is multiplied by this: below 1.0 they push less, which
    // cancels the yaw this chassis picks up when the front out-pushes the rear. 1.0 = no change.
    // Find the value with "Strafe Compensation Test", then set it here; re-run Foresight after any
    // change, since its tuning includes this scale.
    public static double frontStrafeScale = 1.0;
}
