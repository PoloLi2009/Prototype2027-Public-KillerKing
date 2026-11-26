package org.firstinspires.ftc.teamcode.subsystems.pinpoint;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
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
    private Pose2d rawOdometry;
    private Translation2d fieldOffset = new Translation2d();
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
            rawOdometry = null;
            return;
        }

        if (!startPoseSet) {
            driver.setPosition(PinpointConstants.START_POSE);
            startPoseSet = true;
            // The bulk read above still contains the pose from before this write.
            updatePose(PinpointConstants.START_POSE);
            return;
        }

        Pose2D measuredPose = driver.getPosition();
        if (Double.isFinite(measuredPose.getX(DistanceUnit.INCH))
                && Double.isFinite(measuredPose.getY(DistanceUnit.INCH))
                && Double.isFinite(measuredPose.getHeading(AngleUnit.DEGREES))) {
            updatePose(measuredPose);
        } else {
            pose = null;
            rawOdometry = null;
            status = "位姿数据无效";
        }
    }

    private void updatePose(Pose2D measuredPose) {
        rawOdometry = new Pose2d(measuredPose.getX(DistanceUnit.METER),
                measuredPose.getY(DistanceUnit.METER),
                new Rotation2d(measuredPose.getHeading(AngleUnit.RADIANS)));
        updateFieldPose();
    }

    /** Applies a field translation while retaining the measured chassis heading. */
    public void setFieldOffset(Translation2d offset) {
        if (rawOdometry != null) {
            fieldOffset = offset;
            updateFieldPose();
        }
    }

    private void updateFieldPose() {
        pose = new Pose2D(DistanceUnit.METER, rawOdometry.getX() + fieldOffset.getX(),
                rawOdometry.getY() + fieldOffset.getY(), AngleUnit.RADIANS,
                rawOdometry.getRotation().getRadians());
    }

    public Pose2d getRawOdometry() {
        return rawOdometry;
    }

    /** Returns the usable pose in the latest field alignment, or null when odometry is unavailable. */
    public Pose2D getPose() {
        return pose;
    }

    public void addTelemetry(Telemetry telemetry) {
        addTelemetry(telemetry, "X/Y/Heading");
    }

    public void addTelemetry(Telemetry telemetry, String poseLabel) {
        telemetry.addData("Pinpoint (od)", status);
        if (pose != null) {
            telemetry.addData(poseLabel, "%.2f / %.2f in / %.1f°",
                    pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                    pose.getHeading(AngleUnit.DEGREES));
        } else {
            telemetry.addData(poseLabel, "Waiting for pose");
        }
    }
}
