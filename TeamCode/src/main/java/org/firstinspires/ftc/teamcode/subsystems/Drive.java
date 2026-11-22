package org.firstinspires.ftc.teamcode.subsystems;

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
        frontLeft = configureMotor(hardwareMap, "0");
        backLeft = configureMotor(hardwareMap, "1");
        frontRight = configureMotor(hardwareMap, "2");
        backRight = configureMotor(hardwareMap, "3");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
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

        double fl = robotForward + robotStrafe + turn;
        double bl = robotForward - robotStrafe + turn;
        double fr = robotForward - robotStrafe - turn;
        double br = robotForward + robotStrafe - turn;
        double scale = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)),
                Math.max(Math.abs(fr), Math.abs(br))));
        setPowers(fl / scale, bl / scale, fr / scale, br / scale);
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

    private static DcMotorEx configureMotor(HardwareMap hardwareMap, String name) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotor.Direction.REVERSE);
        return motor;
    }
}
