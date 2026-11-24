package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.localization.VisionLocalization;
import org.firstinspires.ftc.teamcode.subsystems.localization.VisionLocalizationConstants;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;

/** Updates localization before the turret command consumes this loop's pose. */
public final class VisionReadCommand extends CommandBase {
    private final LimelightVision vision;
    private final Pinpoint pinpoint;
    private final Turret turret;
    private final VisionLocalization localization;
    private boolean visionForAimEnabled;
    private double estimatedTurretOffset;

    public VisionReadCommand(LimelightVision vision, Pinpoint pinpoint, Turret turret,
                             VisionLocalization localization) {
        this.vision = vision;
        this.pinpoint = pinpoint;
        this.turret = turret;
        this.localization = localization;
        addRequirements(vision, localization);
    }

    @Override
    public void execute() {
        double now = System.nanoTime() / 1e9;
        Pose2D pose = pinpoint.getPose();
        // With the turret centered, the web configuration uses chassis forward as zero.
        // The turret estimate is CCW; the Spinup solver expects CW.
        estimatedTurretOffset = turret.getEstimatedOffsetDegrees(now);
        Pose2d odometry = pose == null ? null : new Pose2d(
                pose.getX(DistanceUnit.METER), pose.getY(DistanceUnit.METER),
                new Rotation2d(pose.getHeading(AngleUnit.RADIANS)));
        localization.update(now, odometry, -estimatedTurretOffset,
                Math.abs(turret.getOffsetDegrees() - estimatedTurretOffset), vision.getNewFrame());
    }

    public void toggleVisionForAim() {
        visionForAimEnabled = !visionForAimEnabled;
    }

    public Pose2D getAimPose() {
        if (usesFusedPose()) {
            Pose2d fused = localization.getEstimatedPosition();
            return new Pose2D(DistanceUnit.METER, fused.getX(), fused.getY(),
                    AngleUnit.RADIANS, fused.getRotation().getRadians());
        }
        return pinpoint.getPose();
    }

    private boolean usesFusedPose() {
        return visionForAimEnabled && localization.hasUsableVisionForAim() && pinpoint.getPose() != null;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Limelight", "%s / P%d / %s", vision.isBlueAlliance() ? "Blue" : "Red",
                vision.getExpectedPipeline(), vision.getStatus());
        telemetry.addData("Vision fusion", "%s (%d accepted / %d rejected)", localization.getStatus(),
                localization.getAcceptedCount(), localization.getRejectedCount());
        telemetry.addData("Aim pose source", getAimPoseStatus());
        telemetry.addData("Camera turret estimate / command", "%.1f° / %.1f°",
                estimatedTurretOffset, turret.getOffsetDegrees());
        addPoseTelemetry(telemetry, "Limelight corrected (last)", localization.getCorrectedVision());
        if (localization.hasFieldPose()) {
            addPoseTelemetry(telemetry, "Pinpoint EKF", localization.getEstimatedPosition());
            addPoseTelemetry(telemetry, "WPILib comparison", localization.getWpiPosition());
            telemetry.addData("Last accepted vision", "%.2f s", localization.getLastVisionAgeSeconds());
        }
        if (Double.isFinite(localization.getHiveTiltDegrees())) {
            telemetry.addData("HIVE tilt", "%.1f°", localization.getHiveTiltDegrees());
        }
    }

    private String getAimPoseStatus() {
        if (pinpoint.getPose() == null) return "Holding (waiting for Pinpoint)";
        if (!visionForAimEnabled) return "Pinpoint (vision preview)";
        if (usesFusedPose()) {
            return localization.getLastVisionAgeSeconds() > VisionLocalizationConstants.VISION_COAST_AFTER_SECONDS
                    ? "EKF (odometry only)" : "EKF (vision recent)";
        }
        return localization.hasFieldPose() ? "Pinpoint (vision expired)" : "Pinpoint (waiting for EKF)";
    }

    private static void addPoseTelemetry(Telemetry telemetry, String label, Pose2d pose) {
        if (pose != null) {
            telemetry.addData(label, "%.1f / %.1f in / %.1f°",
                    DistanceUnit.INCH.fromMeters(pose.getX()), DistanceUnit.INCH.fromMeters(pose.getY()),
                    pose.getRotation().getDegrees());
        }
    }
}
