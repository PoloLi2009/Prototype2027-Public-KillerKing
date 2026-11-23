package org.firstinspires.ftc.teamcode.subsystems.pinpoint;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public final class Pinpoint extends SubsystemBase {
    private final GoBildaPinpointDriver driver;
    private boolean startPoseSet;
    private Pose2D pose;
    private String status = "未配置";

    public Pinpoint(HardwareMap hardwareMap) {
        GoBildaPinpointDriver configuredDriver = null;
        try {
            GoBildaPinpointDriver device = hardwareMap.tryGet(
                    GoBildaPinpointDriver.class, PinpointConstants.DEVICE_NAME);
            if (device != null) {
                device.setOffsets(PinpointConstants.X_OFFSET_MM,
                        PinpointConstants.Y_OFFSET_MM, DistanceUnit.MM);
                device.setEncoderDirections(PinpointConstants.X_ENCODER_DIRECTION,
                        PinpointConstants.Y_ENCODER_DIRECTION);
                device.resetPosAndIMU();
                configuredDriver = device;
                status = "等待 READY";
            }
        } catch (RuntimeException error) {
            status = "初始化失败";
            RobotLog.ee("KillerKingPinpoint", error, "Could not configure Pinpoint '%s'",
                    PinpointConstants.DEVICE_NAME);
        }
        driver = configuredDriver;
    }

    @Override
    public void periodic() {
        if (driver == null) {
            return;
        }

        driver.update();
        GoBildaPinpointDriver.DeviceStatus deviceStatus = driver.getDeviceStatus();
        status = deviceStatus.toString();
        if (deviceStatus != GoBildaPinpointDriver.DeviceStatus.READY) {
            pose = null;
            return;
        }

        if (!startPoseSet) {
            driver.setPosition(PinpointConstants.START_POSE);
            startPoseSet = true;
            // The bulk read above still contains the pose from before this write.
            pose = PinpointConstants.START_POSE;
            return;
        }

        Pose2D measuredPose = driver.getPosition();
        if (Double.isFinite(measuredPose.getX(DistanceUnit.INCH))
                && Double.isFinite(measuredPose.getY(DistanceUnit.INCH))
                && Double.isFinite(measuredPose.getHeading(AngleUnit.DEGREES))) {
            pose = measuredPose;
        } else {
            pose = null;
            status = "位姿数据无效";
        }
    }

    /** Returns the latest usable pose, or null when odometry is unavailable. */
    public Pose2D getPose() {
        return pose;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Pinpoint (od)", status);
        if (pose != null) {
            telemetry.addData("X/Y/Heading", "%.2f / %.2f in / %.1f°",
                    pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                    pose.getHeading(AngleUnit.DEGREES));
        }
    }
}
