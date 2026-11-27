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
    private final Pose2D startPose;
    private boolean startPoseSet;
    private Pose2D pose;
    private Pose2d rawOdometry;
    private int rewrites;
    private String status = "未配置";

    /** Starts at {@link PinpointConstants#START_POSE} (field center). */
    public Pinpoint(HardwareMap hardwareMap) {
        this(hardwareMap, PinpointConstants.START_POSE);
    }

    /** Resets the Pinpoint and writes {@code startPose} once it first reports READY. */
    public Pinpoint(HardwareMap hardwareMap, Pose2D startPose) {
        this.startPose = startPose;
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
            driver.setPosition(startPose);
            startPoseSet = true;
            // The bulk read above still contains the pose from before this write.
            updatePose(startPose);
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
        pose = measuredPose;
    }

    /**
     * Rewrites the Pinpoint device with the current pose corrected by a vision alignment: the current
     * position is rotated by {@code rotation} and shifted by {@code offset}, the heading is turned by
     * {@code rotation}. From then on the device itself reports the corrected pose, so there is no
     * separate field offset. Callers must drop pose history from before the rewrite.
     */
    public void rewritePose(Rotation2d rotation, Translation2d offset) {
        if (driver == null || rawOdometry == null) {
            return;
        }
        Translation2d position = rawOdometry.getTranslation().rotateBy(rotation).plus(offset);
        Pose2D corrected = new Pose2D(DistanceUnit.METER, position.getX(), position.getY(),
                AngleUnit.RADIANS, rawOdometry.getRotation().plus(rotation).getRadians());
        driver.setPosition(corrected);
        rewrites++;
        // This loop's bulk read still holds the old pose; use the written one until the next read.
        updatePose(corrected);
    }

    /** How many times vision has rewritten the Pinpoint pose since this OpMode started. */
    public int getRewriteCount() {
        return rewrites;
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
