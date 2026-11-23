package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public final class Pinpoint extends SubsystemBase {
    private static final String DEVICE_NAME = "od";
    private static final double X_OFFSET_MM = -48.0;
    private static final double Y_OFFSET_MM = -125.0;
    private static final Pose2D START_POSE =
            new Pose2D(DistanceUnit.INCH, 72, 72, AngleUnit.DEGREES, 180);

    private final GoBildaPinpointDriver driver;
    private boolean startPoseSet;
    private Pose2D pose;
    private String status = "未配置";

    public Pinpoint(HardwareMap hardwareMap) {
        GoBildaPinpointDriver configuredDriver = null;
        try {
            GoBildaPinpointDriver device = hardwareMap.tryGet(GoBildaPinpointDriver.class, DEVICE_NAME);
            if (device != null) {
                device.setOffsets(X_OFFSET_MM, Y_OFFSET_MM, DistanceUnit.MM);
                device.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                        GoBildaPinpointDriver.EncoderDirection.FORWARD);
                device.resetPosAndIMU();
                configuredDriver = device;
                status = "等待 READY";
            }
        } catch (RuntimeException error) {
            status = "初始化失败";
            RobotLog.ee("KillerKingPinpoint", error, "Could not configure Pinpoint '%s'", DEVICE_NAME);
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
            driver.setPosition(START_POSE);
            startPoseSet = true;
            // The bulk read above still contains the pose from before this write.
            pose = START_POSE;
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
