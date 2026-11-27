package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.localization.VisionLocalization;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;

/** Updates localization before the turret command consumes this loop's pose. */
public final class VisionReadCommand extends CommandBase {
    private final LimelightVision vision;
    private final Pinpoint pinpoint;
    private final Turret turret;
    private final VisionLocalization localization;
    private double estimatedTurretOffset;

    public VisionReadCommand(LimelightVision vision, Pinpoint pinpoint, Turret turret,
                             VisionLocalization localization) {
        this.vision = vision;
        this.pinpoint = pinpoint;
        this.turret = turret;
        this.localization = localization;
        addRequirements(vision, localization, pinpoint);
    }

    @Override
    public void execute() {
        double now = System.nanoTime() / 1e9;
        // The Limelight web camera yaw is 0; VisionLocalization adds the turret's centered facing.
        // The turret estimate is CCW; the Spinup solver expects CW.
        estimatedTurretOffset = turret.getEstimatedOffsetDegrees(now);
        double trackingError = Math.abs(turret.getOffsetDegrees() - estimatedTurretOffset);
        LimelightVision.Frame frame = vision.getNewFrame();
        VisionLocalization.FieldAlignment alignment = localization.update(now, pinpoint.getRawOdometry(),
                pinpoint.getPose(), -estimatedTurretOffset, trackingError, frame);
        if (alignment != null) {
            // Rewrite the Pinpoint itself, so its own pose is the corrected one from now on.
            pinpoint.rewritePose(alignment.rotation, alignment.translation);
            localization.onOdometryRewritten();
        }
    }

    public Pose2D getAimPose() {
        Pose2D pinpointPose = pinpoint.getPose();
        if (pinpointPose != null) {
            return pinpointPose;
        }
        Pose2d fieldPose = localization.getFreshVisionPose();
        return fieldPose == null ? null : new Pose2D(DistanceUnit.METER,
                fieldPose.getX(), fieldPose.getY(), AngleUnit.RADIANS, fieldPose.getRotation().getRadians());
    }

    /**
     * Makes the next valid Limelight frame overwrite Pinpoint X/Y without the usual confirmation, then
     * corrects the heading from frames taken while the robot and turret are still.
     */
    public void forceVisionAlignment() {
        localization.requestForcedAlignment(System.nanoTime() / 1e9);
    }

    public void addTelemetry(Telemetry telemetry) {
        pinpoint.addTelemetry(telemetry);
        if (pinpoint.getRawOdometry() != null) {
            addPoseTelemetry(telemetry, "Pinpoint raw X/Y/Heading", pinpoint.getRawOdometry());
        }
        telemetry.addData("Limelight", "%s / P%d / %s", vision.isBlueAlliance() ? "Blue" : "Red",
                vision.getExpectedPipeline(), vision.getStatus());
        telemetry.addData("Vision localization", localization.getStatus());
        telemetry.addData("Forced alignment (circle / B)", localization.getForcedAlignmentStatus());
        telemetry.addData("Pinpoint rewrites by vision", pinpoint.getRewriteCount());
        telemetry.addData("Aim pose source", getAimPoseStatus());
        telemetry.addData("Camera turret estimate / command", "%.1f° / %.1f°",
                estimatedTurretOffset, turret.getOffsetDegrees());
        String visionPoseLabel = "Limelight field X/Y/Heading";
        Pose2d correctedVision = localization.getCorrectedVision();
        if (correctedVision != null) {
            addPoseTelemetry(telemetry, visionPoseLabel, correctedVision);
            telemetry.addData("Limelight frame age", "%.0f ms", localization.getVisionAgeSeconds() * 1000.0);
        } else {
            telemetry.addData(visionPoseLabel, "Waiting for pose");
        }
        if (Double.isFinite(localization.getLastPositionErrorMeters())) {
            telemetry.addData("Last vision discrepancy", "%.1f in / %d confirming frames",
                    DistanceUnit.INCH.fromMeters(localization.getLastPositionErrorMeters()),
                    localization.getAlignmentFrames());
        }
        if (localization.getAcceptedCount() > 0) {
            telemetry.addData("Last Pinpoint alignment", "%.2f s ago", localization.getLastAlignmentAgeSeconds());
        }
        if (Double.isFinite(localization.getHiveTiltDegrees())) {
            telemetry.addData("HIVE tilt", "%.1f°", localization.getHiveTiltDegrees());
        }
    }

    private String getAimPoseStatus() {
        if (pinpoint.getPose() == null) {
            if (localization.getFreshVisionPose() != null) {
                return "Limelight only (Pinpoint unavailable)";
            }
            return "Holding (waiting for pose)";
        }
        if (localization.getAcceptedCount() == 0) {
            return "Pinpoint (preset start)";
        }
        return localization.isHeadingCalibrated()
                ? "Pinpoint (vision X/Y and heading)" : "Pinpoint (vision X/Y, Pinpoint heading)";
    }

    private static void addPoseTelemetry(Telemetry telemetry, String label, Pose2d pose) {
        telemetry.addData(label, "%.1f / %.1f in / %.1f°",
                DistanceUnit.INCH.fromMeters(pose.getX()), DistanceUnit.INCH.fromMeters(pose.getY()),
                pose.getRotation().getDegrees());
    }
}
