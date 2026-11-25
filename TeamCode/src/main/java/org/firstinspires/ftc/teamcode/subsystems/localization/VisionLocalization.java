package org.firstinspires.ftc.teamcode.subsystems.localization;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.subsystems.localization.fusion.PoseHistory;
import org.firstinspires.ftc.teamcode.subsystems.vision.HiveFlatPoseSolver;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;

/** Resolves HIVE observations in the shared field frame and confirms odometry corrections. */
public final class VisionLocalization extends SubsystemBase {
    public enum HiveMotionState {
        STILL, TILTING_TO_OTHER_SIDE, UNKNOWN
    }

    private static final double HIVE_MOTION_WINDOW_SECONDS = 0.15;
    private static final double HIVE_STILL_CHANGE_DEGREES = 1.0;
    private static final double HIVE_MOTION_CHANGE_DEGREES = 3.0;
    private static final double HIVE_MIN_SIDE_DEGREES = 3.0;
    private static final Pose2d NO_ODOMETRY_MOTION = new Pose2d();

    private final PoseHistory history = new PoseHistory();
    private boolean hasOdometryHistory;
    private Pose2d correctedVision;
    private String status = "Waiting for qualifying vision frame";
    private double tilt = Double.NaN;
    private double motionWindowStart = Double.NaN;
    private double motionWindowTilt;
    private double motionWindowMin;
    private double motionWindowMax;
    private double stableHiveSide;
    private HiveMotionState hiveMotionState = HiveMotionState.UNKNOWN;
    private double lastFrameTime = Double.NEGATIVE_INFINITY;
    private double lastAlignmentTime = Double.NaN;
    private double lastUpdateTime = Double.NaN;
    private double turretReadyTime = Double.NaN;
    private double lastPositionErrorMeters = Double.NaN;
    private PoseHistory.Sample alignmentStart;
    private Translation2d alignmentStartOffset;
    private int alignmentFrames;
    private int lastPipeline = -1;
    private int accepted;
    private int rejected;

    /** Returns the confirmed translation from raw Pinpoint coordinates to field coordinates. */
    public Translation2d update(double now, Pose2d odometry, Pose2D currentFieldPose, double turretClockwiseDegrees,
                         double turretTrackingErrorDegrees, LimelightVision.Frame frame) {
        lastUpdateTime = now;
        boolean hasOdometry = odometry != null && currentFieldPose != null;
        if (hasOdometry != hasOdometryHistory) {
            history.clear();
            resetAlignmentConfirmation();
            hasOdometryHistory = hasOdometry;
        }
        // Without Pinpoint, keep real turret samples for the independent visual estimate.
        recordSample(now, hasOdometry ? odometry : NO_ODOMETRY_MOTION,
                turretClockwiseDegrees, turretTrackingErrorDegrees);
        PoseHistory.Sample past = readObservation(frame, hasOdometry);
        if (!hasOdometry || past == null) {
            return null;
        }

        // Compare positions at exposure so motion during camera latency does not become a correction.
        Translation2d offset = correctedVision.getTranslation().minus(past.odometry.getTranslation());
        Translation2d currentOffset = new Translation2d(currentFieldPose.getX(DistanceUnit.METER),
                currentFieldPose.getY(DistanceUnit.METER)).minus(odometry.getTranslation());
        lastPositionErrorMeters = offset.getDistance(currentOffset);
        if (lastPositionErrorMeters < VisionLocalizationConstants.MIN_ALIGNMENT_POSITION_ERROR_METERS) {
            resetAlignmentConfirmation();
            status = "Pinpoint within vision threshold";
            return null;
        }
        if (!confirmedAlignment(past, offset)) {
            status = "Confirming vision discrepancy";
            return null;
        }

        resetAlignmentConfirmation();
        lastAlignmentTime = past.time;
        accepted++;
        status = "Pinpoint X/Y aligned to Limelight";
        return offset;
    }

    private void recordSample(double now, Pose2d odometry, double turretClockwiseDegrees,
                              double turretTrackingErrorDegrees) {
        if (!Double.isFinite(turretReadyTime)) {
            turretReadyTime = now + VisionLocalizationConstants.TURRET_INITIAL_SETTLE_SECONDS;
        }
        history.add(now, odometry, turretClockwiseDegrees, turretTrackingErrorDegrees);
    }

    /** Validates a new frame and stores its field pose at the time of exposure. */
    private PoseHistory.Sample readObservation(LimelightVision.Frame frame, boolean usePinpointHeadingForPivot) {
        if (frame == null) {
            return null;
        }

        double capture = frame.approximateCaptureNanos / 1e9;
        if (capture <= lastFrameTime) {
            reject("Vision frame out of order");
            return null;
        }
        PoseHistory.Sample past = history.sample(capture);
        if (past == null) {
            reject("Capture outside pose history");
            return null;
        }
        if (capture < turretReadyTime) {
            reject("Waiting for initial turret center");
            return null;
        }
        PoseHistory.Sample earlier = history.sample(capture - VisionLocalizationConstants.TURRET_SAMPLE_SECONDS);
        if (earlier == null) {
            reject("Waiting for turret angle history");
            return null;
        }
        double turretSpeed = Math.abs(past.turretDegrees - earlier.turretDegrees)
                / VisionLocalizationConstants.TURRET_SAMPLE_SECONDS;
        if (turretSpeed > VisionLocalizationConstants.maxTurretSpeedDegreesPerSecond
                || past.turretTrackingErrorDegrees > VisionLocalizationConstants.MAX_TURRET_TRACKING_ERROR_DEGREES) {
            reject("Turret motion too fast at exposure");
            return null;
        }

        Position position = frame.rawPose.getPosition().toUnit(DistanceUnit.METER);
        HiveFlatPoseSolver.Result result = HiveFlatPoseSolver.solve(position.x, position.y, position.z,
                frame.rawPose.getOrientation().getYaw(AngleUnit.RADIANS),
                frame.rawPose.getOrientation().getPitch(AngleUnit.RADIANS),
                frame.rawPose.getOrientation().getRoll(AngleUnit.RADIANS), past.turretDegrees);
        if (result == null) {
            tilt = Double.NaN;
            clearHiveMotion();
            reject("HIVE geometry / floor / level mismatch");
            return null;
        }

        Pose2d fieldPoseAtCapture = toPinpointFrame(result.chassisPose,
                usePinpointHeadingForPivot ? past.odometry.getRotation() : null);
        if (!isWithinField(fieldPoseAtCapture)) {
            tilt = Double.NaN;
            clearHiveMotion();
            reject("Corrected vision pose outside field bounds");
            return null;
        }
        correctedVision = fieldPoseAtCapture;
        tilt = result.hiveTiltDegrees;
        updateHiveMotion(capture, frame.pipeline);
        if (frame.pipeline != lastPipeline
                || capture - lastFrameTime > VisionLocalizationConstants.MAX_FRAME_GAP_SECONDS) {
            resetAlignmentConfirmation();
        }
        lastPipeline = frame.pipeline;
        lastFrameTime = capture;
        return past;
    }

    private boolean confirmedAlignment(PoseHistory.Sample sample, Translation2d offset) {
        if (alignmentStart == null || alignmentStartOffset.getDistance(offset)
                > VisionLocalizationConstants.ALIGNMENT_POSITION_TOLERANCE_METERS) {
            alignmentStart = sample;
            alignmentStartOffset = offset;
            alignmentFrames = 1;
        } else {
            alignmentFrames++;
        }
        return alignmentFrames >= VisionLocalizationConstants.MIN_ALIGNMENT_FRAMES
                && sample.time - alignmentStart.time >= VisionLocalizationConstants.ALIGNMENT_CONFIRMATION_SECONDS;
    }

    private void resetAlignmentConfirmation() {
        alignmentStart = null;
        alignmentStartOffset = null;
        alignmentFrames = 0;
    }

    private static Pose2d toPinpointFrame(Pose2d mapPose, Rotation2d pinpointHeading) {
        Rotation2d axes = Rotation2d.fromDegrees(VisionLocalizationConstants.MAP_TO_PINPOINT_YAW_DEGREES);
        double center = DistanceUnit.INCH.toMeters(VisionLocalizationConstants.FIELD_CENTER_INCHES);
        Rotation2d heading = mapPose.getRotation().plus(axes);
        Translation2d position = mapPose.getTranslation().rotateBy(axes)
                .plus(new Translation2d(center, center));
        Translation2d pivotOffset = new Translation2d(VisionLocalizationConstants.TURRET_PIVOT_FORWARD_METERS,
                VisionLocalizationConstants.TURRET_PIVOT_LEFT_METERS);
        return new Pose2d(position.minus(pivotOffset.rotateBy(pinpointHeading != null ? pinpointHeading : heading)), heading);
    }

    private static boolean isWithinField(Pose2d pose) {
        double margin = VisionLocalizationConstants.FIELD_BOUNDARY_MARGIN_METERS;
        double max = DistanceUnit.INCH.toMeters(2 * VisionLocalizationConstants.FIELD_CENTER_INCHES) + margin;
        return pose.getX() >= -margin && pose.getX() <= max
                && pose.getY() >= -margin && pose.getY() <= max;
    }

    private void clearHiveMotion() {
        motionWindowStart = Double.NaN;
        stableHiveSide = 0;
        hiveMotionState = HiveMotionState.UNKNOWN;
    }

    private void updateHiveMotion(double capture, int pipeline) {
        if (pipeline != lastPipeline || capture - lastFrameTime
                > VisionLocalizationConstants.MAX_FRAME_GAP_SECONDS) {
            clearHiveMotion();
        }
        if (Double.isNaN(motionWindowStart)) {
            startHiveMotionWindow(capture);
            return;
        }

        motionWindowMin = Math.min(motionWindowMin, tilt);
        motionWindowMax = Math.max(motionWindowMax, tilt);
        if (capture - motionWindowStart < HIVE_MOTION_WINDOW_SECONDS) {
            return;
        }

        double change = tilt - motionWindowTilt;
        if (motionWindowMax - motionWindowMin <= HIVE_STILL_CHANGE_DEGREES) {
            hiveMotionState = HiveMotionState.STILL;
            stableHiveSide = Math.abs(tilt) >= HIVE_MIN_SIDE_DEGREES ? Math.signum(tilt) : 0;
        } else if (change * stableHiveSide <= -HIVE_MOTION_CHANGE_DEGREES) {
            hiveMotionState = HiveMotionState.TILTING_TO_OTHER_SIDE;
        } else {
            hiveMotionState = HiveMotionState.UNKNOWN;
        }
        startHiveMotionWindow(capture);
    }

    private void startHiveMotionWindow(double capture) {
        motionWindowStart = capture;
        motionWindowTilt = tilt;
        motionWindowMin = tilt;
        motionWindowMax = tilt;
    }

    private void reject(String reason) {
        resetAlignmentConfirmation();
        status = reason;
        rejected++;
    }

    public Pose2d getCorrectedVision() { return correctedVision; }
    public Pose2d getFreshVisionPose() {
        return getVisionAgeSeconds() * 1000.0 <= VisionConstants.maxFrameAgeMs ? correctedVision : null;
    }
    public double getVisionAgeSeconds() { return lastUpdateTime - lastFrameTime; }
    /** Latest valid tilt, in degrees, or NaN when the observation is stale. */
    public double getHiveTiltDegrees() {
        return lastUpdateTime - lastFrameTime
                <= VisionLocalizationConstants.MAX_FRAME_GAP_SECONDS ? tilt : Double.NaN;
    }

    public HiveMotionState getHiveMotionState() {
        return Double.isFinite(getHiveTiltDegrees()) ? hiveMotionState : HiveMotionState.UNKNOWN;
    }
    public double getLastAlignmentAgeSeconds() { return lastUpdateTime - lastAlignmentTime; }
    public double getLastPositionErrorMeters() { return lastPositionErrorMeters; }
    public int getAlignmentFrames() { return alignmentFrames; }
    public String getStatus() { return status; }
    public int getAcceptedCount() { return accepted; }
    public int getRejectedCount() { return rejected; }
}
