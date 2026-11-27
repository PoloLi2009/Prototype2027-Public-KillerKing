package org.firstinspires.ftc.teamcode.subsystems.localization;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.util.RobotLog;

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

    /**
     * Raw Pinpoint to field: the raw position is rotated by {@link #rotation}, then shifted by
     * {@link #translation}; the field heading is the raw heading plus {@link #rotation}.
     */
    public static final class FieldAlignment {
        public final Rotation2d rotation;
        public final Translation2d translation;

        FieldAlignment(Rotation2d rotation, Translation2d translation) {
            this.rotation = rotation;
            this.translation = translation;
        }
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
    private double forcedAlignmentRequestTime = Double.NaN;
    private String forcedAlignmentStatus = "Not requested";
    private boolean forcedPositionApplied;
    private final double[] forcedHeadingErrors =
            new double[VisionLocalizationConstants.FORCED_HEADING_FRAMES];
    private int forcedHeadingFrames;
    private boolean lastFrameStill;
    private boolean headingCalibrated;
    private PoseHistory.Sample alignmentStart;
    private Translation2d alignmentStartOffset;
    private int alignmentFrames;
    private int lastPipeline = -1;
    private int accepted;
    private int rejected;

    /**
     * Returns a new raw-Pinpoint-to-field alignment when vision corrects odometry, else null. The
     * automatic correction only moves X/Y and keeps the current heading correction; a forced
     * alignment (circle / B) also measures the heading.
     */
    public FieldAlignment update(double now, Pose2d odometry, Pose2D currentFieldPose, double turretClockwiseDegrees,
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
        // The alignment in use, recovered from this loop's raw and field poses.
        Rotation2d rotation = hasOdometry
                ? new Rotation2d(currentFieldPose.getHeading(AngleUnit.RADIANS)).minus(odometry.getRotation())
                : null;
        PoseHistory.Sample past = readObservation(frame, rotation);
        expireForcedAlignment(now);
        if (!hasOdometry || past == null) {
            return null;
        }

        // Compare positions at exposure so motion during camera latency does not become a correction.
        Translation2d offset = offsetAt(past, rotation);
        Translation2d currentOffset = new Translation2d(currentFieldPose.getX(DistanceUnit.METER),
                currentFieldPose.getY(DistanceUnit.METER)).minus(odometry.getTranslation().rotateBy(rotation));
        lastPositionErrorMeters = offset.getDistance(currentOffset);
        if (Double.isFinite(forcedAlignmentRequestTime)) {
            return forcedAlignment(past, rotation, offset);
        }
        if (lastPositionErrorMeters < VisionLocalizationConstants.MIN_ALIGNMENT_POSITION_ERROR_METERS) {
            resetAlignmentConfirmation();
            status = "Pinpoint within vision threshold";
            return null;
        }
        if (!confirmedAlignment(past, offset)) {
            status = "Confirming vision discrepancy";
            return null;
        }
        return applyAlignment(past, new FieldAlignment(rotation, offset), "Pinpoint X/Y aligned to Limelight");
    }

    /**
     * Applies the next valid Limelight frame to Pinpoint X/Y, skipping the distance threshold and the
     * multi-frame confirmation, then measures the heading from
     * {@link VisionLocalizationConstants#FORCED_HEADING_FRAMES} frames taken with the turret and the
     * chassis still, and applies X/Y and heading together. The request is dropped if no valid frame
     * arrives within {@link VisionLocalizationConstants#FORCED_ALIGNMENT_TIMEOUT_SECONDS}; the heading
     * step gives up after {@link VisionLocalizationConstants#FORCED_HEADING_TIMEOUT_SECONDS}.
     */
    public void requestForcedAlignment(double now) {
        forcedAlignmentRequestTime = now;
        forcedPositionApplied = false;
        forcedHeadingFrames = 0;
        forcedAlignmentStatus = "Waiting for valid Limelight frame";
    }

    private FieldAlignment forcedAlignment(PoseHistory.Sample past, Rotation2d rotation, Translation2d offset) {
        FieldAlignment positionOnly = null;
        if (!forcedPositionApplied) {
            forcedPositionApplied = true;
            forcedAlignmentStatus = "X/Y applied; measuring heading (hold still)";
            positionOnly = applyAlignment(past, new FieldAlignment(rotation, offset),
                    "Pinpoint X/Y forced to Limelight");
        }
        if (!lastFrameStill) {
            return positionOnly;
        }

        // Heading correction this frame implies: vision field heading minus raw Pinpoint heading.
        forcedHeadingErrors[forcedHeadingFrames++] =
                correctedVision.getRotation().minus(past.odometry.getRotation()).getRadians();
        if (forcedHeadingFrames < VisionLocalizationConstants.FORCED_HEADING_FRAMES) {
            return positionOnly;
        }
        forcedHeadingFrames = 0;
        double sin = 0;
        double cos = 0;
        for (double error : forcedHeadingErrors) {
            sin += Math.sin(error);
            cos += Math.cos(error);
        }
        Rotation2d measured = new Rotation2d(Math.atan2(sin, cos));
        double spread = 0;
        for (double error : forcedHeadingErrors) {
            spread = Math.max(spread, Math.abs(new Rotation2d(error).minus(measured).getDegrees()));
        }
        if (spread > VisionLocalizationConstants.FORCED_HEADING_MAX_SPREAD_DEGREES) {
            forcedAlignmentStatus = String.format("X/Y applied; heading frames %.1f° apart, retrying", spread);
            return positionOnly;
        }

        forcedAlignmentRequestTime = Double.NaN;
        headingCalibrated = true;
        double change = measured.minus(rotation).getDegrees();
        forcedAlignmentStatus = String.format("Applied X/Y and heading (%+.1f°)", change);
        RobotLog.ii("KillerKingVision", "Forced heading: %+.1f deg change, correction %.1f deg, spread %.1f deg",
                change, measured.getDegrees(), spread);
        return applyAlignment(past, new FieldAlignment(measured, offsetAt(past, measured)),
                "Pinpoint X/Y and heading forced to Limelight");
    }

    /**
     * Call after the odometry device was rewritten with an alignment from {@link #update}: the raw pose
     * jumped, so the stored history (and any half-confirmed automatic correction) no longer matches it.
     * Frames captured before the rewrite then fall outside the history and are dropped; a forced
     * heading measurement in progress carries on with new frames.
     */
    public void onOdometryRewritten() {
        history.clear();
        resetAlignmentConfirmation();
    }

    private void expireForcedAlignment(double now) {
        if (!Double.isFinite(forcedAlignmentRequestTime)) {
            return;
        }
        double age = now - forcedAlignmentRequestTime;
        if (!forcedPositionApplied && age > VisionLocalizationConstants.FORCED_ALIGNMENT_TIMEOUT_SECONDS) {
            forcedAlignmentRequestTime = Double.NaN;
            forcedAlignmentStatus = "Timed out: no valid Limelight frame";
        } else if (forcedPositionApplied && age > VisionLocalizationConstants.FORCED_HEADING_TIMEOUT_SECONDS) {
            forcedAlignmentRequestTime = Double.NaN;
            forcedAlignmentStatus = "X/Y applied; heading timed out (stop and let the turret settle)";
        }
    }

    /** Translation that puts the raw pose at {@code past} on the vision pose, for {@code rotation}. */
    private Translation2d offsetAt(PoseHistory.Sample past, Rotation2d rotation) {
        return correctedVision.getTranslation().minus(past.odometry.getTranslation().rotateBy(rotation));
    }

    private FieldAlignment applyAlignment(PoseHistory.Sample past, FieldAlignment alignment, String newStatus) {
        resetAlignmentConfirmation();
        lastAlignmentTime = past.time;
        accepted++;
        status = newStatus;
        return alignment;
    }

    private void recordSample(double now, Pose2d odometry, double turretClockwiseDegrees,
                              double turretTrackingErrorDegrees) {
        if (!Double.isFinite(turretReadyTime)) {
            turretReadyTime = now + VisionLocalizationConstants.TURRET_INITIAL_SETTLE_SECONDS;
        }
        history.add(now, odometry, turretClockwiseDegrees, turretTrackingErrorDegrees);
    }

    /**
     * Validates a new frame and stores its field pose at the time of exposure. {@code rawToField} is
     * the heading correction in use, or null without odometry.
     */
    private PoseHistory.Sample readObservation(LimelightVision.Frame frame, Rotation2d rawToField) {
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

        // The solver's heading is the camera's (turret yaw minus the turret angle); turn it to the
        // chassis heading. Position and HIVE tilt do not depend on it.
        Pose2d chassisInMap = new Pose2d(result.chassisPose.getTranslation(), result.chassisPose.getRotation()
                .minus(Rotation2d.fromDegrees(VisionLocalizationConstants.TURRET_CENTER_CAMERA_YAW_DEGREES)));
        Pose2d fieldPoseAtCapture = toPinpointFrame(chassisInMap,
                rawToField != null ? past.odometry.getRotation().plus(rawToField) : null);
        if (!isWithinField(fieldPoseAtCapture)) {
            tilt = Double.NaN;
            clearHiveMotion();
            reject("Corrected vision pose outside field bounds");
            return null;
        }
        correctedVision = fieldPoseAtCapture;
        lastFrameStill = wasStill(capture, past);
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

    /**
     * Whether a frame can be trusted for heading: the turret estimate had reached its command, and over
     * the preceding {@link VisionLocalizationConstants#FORCED_HEADING_STILL_SECONDS} neither the turret
     * estimate nor the chassis heading moved more than the limits.
     */
    private boolean wasStill(double capture, PoseHistory.Sample past) {
        PoseHistory.Sample before = history.sample(capture - VisionLocalizationConstants.FORCED_HEADING_STILL_SECONDS);
        return before != null
                && past.turretTrackingErrorDegrees <= VisionLocalizationConstants.FORCED_HEADING_MAX_TURRET_ERROR_DEGREES
                && Math.abs(past.turretDegrees - before.turretDegrees)
                        <= VisionLocalizationConstants.FORCED_HEADING_MAX_TURRET_CHANGE_DEGREES
                && Math.abs(past.odometry.getRotation().minus(before.odometry.getRotation()).getDegrees())
                        <= VisionLocalizationConstants.FORCED_HEADING_MAX_CHASSIS_TURN_DEGREES;
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
    public String getForcedAlignmentStatus() { return forcedAlignmentStatus; }
    /** Whether a forced alignment has corrected the heading since this OpMode started. */
    public boolean isHeadingCalibrated() { return headingCalibrated; }
    public int getAcceptedCount() { return accepted; }
    public int getRejectedCount() { return rejected; }
}
