package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public final class Drive extends SubsystemBase {
    private final DcMotorEx frontLeft;
    private final DcMotorEx backLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backRight;
    private final IMU imu;

    public Drive(HardwareMap hardwareMap) {
        frontLeft = configureMotor(hardwareMap, DriveConstants.FRONT_LEFT_MOTOR_NAME,
                DriveConstants.LEFT_MOTOR_DIRECTION);
        backLeft = configureMotor(hardwareMap, DriveConstants.BACK_LEFT_MOTOR_NAME,
                DriveConstants.LEFT_MOTOR_DIRECTION);
        frontRight = configureMotor(hardwareMap, DriveConstants.FRONT_RIGHT_MOTOR_NAME,
                DriveConstants.RIGHT_MOTOR_DIRECTION);
        backRight = configureMotor(hardwareMap, DriveConstants.BACK_RIGHT_MOTOR_NAME,
                DriveConstants.RIGHT_MOTOR_DIRECTION);

        imu = hardwareMap.get(IMU.class, DriveConstants.IMU_NAME);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.IMU_LOGO_DIRECTION,
                DriveConstants.IMU_USB_DIRECTION)));
        resetYaw();
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public void driveFieldCentric(double forward, double strafe, double turn) {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double cos = Math.cos(yaw);
        double sin = Math.sin(yaw);
        double robotStrafe = strafe * cos + forward * sin;
        double robotForward = -strafe * sin + forward * cos;
        driveRobotCentric(robotForward, robotStrafe, turn);
    }

    /**
     * Robot-relative drive; strafe is right-positive. The front wheels' share of the strafe is scaled
     * by {@link DriveConstants#frontStrafeScale} to cancel the yaw this chassis picks up when it
     * strafes. Forward and turn are unchanged.
     */
    public void driveRobotCentric(double forward, double strafe, double turn) {
        double frontStrafe = strafe * DriveConstants.frontStrafeScale;
        double fl = forward + frontStrafe + turn;
        double bl = forward - strafe + turn;
        double fr = forward - frontStrafe - turn;
        double br = forward + strafe - turn;
        double scale = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                Math.max(Math.abs(fr), Math.abs(br))));
        setPowers(fl / scale, bl / scale, fr / scale, br / scale);
    }

    public double getYawDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("IMU Yaw (°)", "%.1f",
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("底盘 FL/BL/FR/BR", "%.2f / %.2f / %.2f / %.2f",
                frontLeft.getPower(), backLeft.getPower(), frontRight.getPower(), backRight.getPower());
    }

    private void setPowers(double fl, double bl, double fr, double br) {
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    private static DcMotorEx configureMotor(HardwareMap hardwareMap, String name, DcMotor.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setMode(DriveConstants.MOTOR_RUN_MODE);
        motor.setZeroPowerBehavior(DriveConstants.ZERO_POWER_BEHAVIOR);
        motor.setDirection(direction);
        return motor;
    }
}
