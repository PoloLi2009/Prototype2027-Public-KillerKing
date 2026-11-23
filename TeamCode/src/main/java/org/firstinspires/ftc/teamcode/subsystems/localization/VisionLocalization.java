package org.firstinspires.ftc.teamcode.subsystems.localization;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.subsystems.localization.fusion.PinpointEkf;
import org.firstinspires.ftc.teamcode.subsystems.localization.fusion.PoseHistory;
import org.firstinspires.ftc.teamcode.subsystems.localization.fusion.WpiPinpointPoseEstimator;
import org.firstinspires.ftc.teamcode.subsystems.vision.HiveFlatPoseSolver;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;

/** Spinup HIVE correction and delayed pose fusion, in Killer King's field frame. */
public final class VisionLocalization extends SubsystemBase {
    private static final double TURRET_SAMPLE_SECONDS = 0.020;
    private static final double TURRET_ANGLE_STEP_DEGREES = 1.0;
    private static final double HIVE_STABLE_SECONDS = 0.15;
    private static final double MAX_TILT_STEP_DEGREES = 2.0;
    private static final double MAX_FRAME_GAP_SECONDS = 0.2;

    private final PoseHistory history = new PoseHistory();
    private final WpiPinpointPoseEstimator wpi = new WpiPinpointPoseEstimator();
    private final PinpointEkf ekf = new PinpointEkf();
    private Pose2d correctedVision;
    private String status = "Waiting for Pinpoint / vision";
    private double tilt = Double.NaN;
    private double lastTilt = Double.NaN;
    private double stableSince;
    private double lastFrameTime = Double.NEGATIVE_INFINITY;
    private double lastAcceptedTime = Double.NaN;
    private double lastTurretDegrees = Double.NaN;
    private double lastTurretMotionTime;
    private int lastPipeline = -1;
    private int accepted;
    private int rejected;

    public void clear() {
        history.clear();
        wpi.reset();
        ekf.reset();
        correctedVision = null;
        tilt = Double.NaN;
        lastTilt = Double.NaN;
        lastTurretDegrees = Double.NaN;
        lastAcceptedTime = Double.NaN;
        lastFrameTime = Double.NEGATIVE_INFINITY;
        lastPipeline = -1;
        status = "Waiting for Pinpoint / vision";
    }

    public void update(double now, Pose2d odometry, double turretClockwiseDegrees,
                       LimelightVision.Frame frame) {
        history.add(now, odometry, turretClockwiseDegrees);
        wpi.updateWithTime(now, odometry);
        ekf.updateWithTime(now, odometry);

        if (!Double.isFinite(lastTurretDegrees)
                || Math.abs(turretClockwiseDegrees - lastTurretDegrees) >= TURRET_ANGLE_STEP_DEGREES) {
            lastTurretDegrees = turretClockwiseDegrees;
            lastTurretMotionTime = now;
        }

        if (frame == null) {
            return;
        }

        double capture = frame.approximateCaptureNanos / 1e9;
        PoseHistory.Sample past = history.sample(capture);
        if (past == null) {
            reject("Capture outside Pinpoint history");
            return;
        }
        if (capture < lastTurretMotionTime + VisionConstants.turretSettleSeconds) {
            reject("Waiting for turret to settle");
            return;
        }
        PoseHistory.Sample earlier = history.sample(capture - TURRET_SAMPLE_SECONDS);
        if (earlier == null || Math.abs(past.turretDegrees - earlier.turretDegrees)
                / TURRET_SAMPLE_SECONDS > VisionConstants.maxTurretSpeedDegreesPerSecond) {
            reject("Turret motion at exposure");
            return;
        }

        Position position = frame.rawPose.getPosition().toUnit(DistanceUnit.METER);
        HiveFlatPoseSolver.Result result = HiveFlatPoseSolver.solve(position.x, position.y, position.z,
                frame.rawPose.getOrientation().getYaw(AngleUnit.RADIANS),
                frame.rawPose.getOrientation().getPitch(AngleUnit.RADIANS),
                frame.rawPose.getOrientation().getRoll(AngleUnit.RADIANS), past.turretDegrees);
        if (result == null) {
            correctedVision = null;
            tilt = Double.NaN;
            lastTilt = Double.NaN;
            reject("HIVE geometry / floor / level mismatch");
            return;
        }

        tilt = result.hiveTiltDegrees;
        correctedVision = toPinpointFrame(result.chassisPose);
        if (frame.pipeline != lastPipeline || !Double.isFinite(lastTilt)
                || capture - lastFrameTime > MAX_FRAME_GAP_SECONDS
                || Math.abs(tilt - lastTilt) > MAX_TILT_STEP_DEGREES) {
            stableSince = capture;
        }
        lastPipeline = frame.pipeline;
        lastTilt = tilt;
        lastFrameTime = capture;
        if (capture - stableSince < HIVE_STABLE_SECONDS) {
            reject("Waiting for stable HIVE tilt");
            return;
        }

        // Spinup seeds directly from its first accepted pose. Check it against Killer King's
        // known odometry frame first so an uncalibrated mount or axis cannot seed a large jump.
        if (!ekf.isSeeded() && (correctedVision.getTranslation().getDistance(past.odometry.getTranslation())
                > VisionConstants.maxSeedDifferenceMeters
                || Math.abs(correctedVision.getRotation().minus(past.odometry.getRotation()).getDegrees())
                > VisionConstants.maxSeedHeadingErrorDegrees)) {
            reject("Vision and Pinpoint disagree at initialization");
            return;
        }

        double distance = frame.averageDistanceMeters;
        double xyStd = VisionConstants.minimumVisionXyStdMeters
                + VisionConstants.visionDistanceNoiseCoefficient * distance * distance
                / Math.sqrt(frame.tagCount);
        double headingStd = Math.toRadians(VisionConstants.visionHeadingStdDegrees);
        if (!Double.isFinite(xyStd)) {
            reject("Invalid measurement noise");
            return;
        }

        boolean wpiAccepted = wpi.addVisionMeasurement(correctedVision, capture, xyStd, headingStd);
        boolean ekfAccepted = ekf.addVisionMeasurement(correctedVision, capture, xyStd,
                headingStd, past.odometry);
        if (ekfAccepted) {
            accepted++;
            lastAcceptedTime = now;
        } else {
            rejected++;
        }
        status = "WPI " + (wpiAccepted ? "accepted" : "rejected") + " / EKF " + ekf.getStatus();
    }

    private static Pose2d toPinpointFrame(Pose2d mapPose) {
        Rotation2d axes = Rotation2d.fromDegrees(VisionConstants.mapToPinpointYawDegrees);
        double center = DistanceUnit.INCH.toMeters(VisionConstants.fieldCenterInches);
        Rotation2d heading = mapPose.getRotation().plus(axes);
        Translation2d position = mapPose.getTranslation().rotateBy(axes)
                .plus(new Translation2d(center, center));
        Translation2d pivotOffset = new Translation2d(VisionConstants.turretPivotForwardMeters,
                VisionConstants.turretPivotLeftMeters);
        return new Pose2d(position.minus(pivotOffset.rotateBy(heading)), heading);
    }

    private void reject(String reason) {
        status = reason;
        rejected++;
    }

    public boolean hasFieldPose() { return ekf.isSeeded(); }
    public Pose2d getEstimatedPosition() { return ekf.getEstimatedPosition(); }
    public Pose2d getWpiPosition() { return wpi.getEstimatedPosition(); }
    public Pose2d getCorrectedVision() { return correctedVision; }
    public double getHiveTiltDegrees() { return tilt; }
    public double getLastVisionAgeSeconds() { return System.nanoTime() / 1e9 - lastAcceptedTime; }
    public String getStatus() { return status; }
    public int getAcceptedCount() { return accepted; }
    public int getRejectedCount() { return rejected; }
}
