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

/** Spinup HIVE correction and delayed pose fusion, in Killer King's field frame. */
public final class VisionLocalization extends SubsystemBase {
    public enum HiveMotionState {
        STILL, TILTING_TO_OTHER_SIDE, UNKNOWN
    }

    private static final double HIVE_MOTION_WINDOW_SECONDS = 0.15;
    private static final double HIVE_STILL_CHANGE_DEGREES = 1.0;
    private static final double HIVE_MOTION_CHANGE_DEGREES = 3.0;
    private static final double HIVE_MIN_SIDE_DEGREES = 3.0;

    private final PoseHistory history = new PoseHistory();
    private final WpiPinpointPoseEstimator wpi = new WpiPinpointPoseEstimator();
    private final PinpointEkf ekf = new PinpointEkf();
    private Pose2d correctedVision;
    private String status = "Waiting for Pinpoint / vision";
    private double tilt = Double.NaN;
    private double motionWindowStart = Double.NaN;
    private double motionWindowTilt;
    private double motionWindowMin;
    private double motionWindowMax;
    private double stableHiveSide;
    private HiveMotionState hiveMotionState = HiveMotionState.UNKNOWN;
    private double lastTilt = Double.NaN;
    private double stableSince;
    private double lastFrameTime = Double.NEGATIVE_INFINITY;
    private double lastAcceptedTime = Double.NaN;
    private double lastUpdateTime = Double.NaN;
    private double lastOdometryTime = Double.NaN;
    private double turretReadyTime;
    private double earliestCaptureTime = Double.NEGATIVE_INFINITY;
    private boolean pinpointMissing;
    private Pose2d relocalizationPose;
    private PoseHistory.Sample relocalizationSample;
    private int relocalizationFrames;
    private int lastPipeline = -1;
    private int accepted;
    private int rejected;

    private void clear() {
        history.clear();
        wpi.reset();
        ekf.reset();
        correctedVision = null;
        tilt = Double.NaN;
        clearHiveMotion();
        lastTilt = Double.NaN;
        lastOdometryTime = Double.NaN;
        lastAcceptedTime = Double.NaN;
        lastFrameTime = Double.NEGATIVE_INFINITY;
        lastPipeline = -1;
        resetRelocalization();
    }

    /** A null odometry sample pauses vision updates and preserves the estimator during brief gaps. */
    public void update(double now, Pose2d odometry, double turretClockwiseDegrees,
                       double turretTrackingErrorDegrees, LimelightVision.Frame frame) {
        lastUpdateTime = now;
        if (odometry == null) {
            pinpointMissing = true;
            if (Double.isFinite(lastOdometryTime)
                    && now - lastOdometryTime > VisionLocalizationConstants.PINPOINT_DROPOUT_GRACE_SECONDS) {
                clear();
            }
            status = Double.isFinite(lastOdometryTime)
                    ? "Pinpoint gap; holding fusion" : "Waiting for Pinpoint";
            return;
        }

        if (pinpointMissing && now - lastOdometryTime > VisionLocalizationConstants.PINPOINT_DROPOUT_GRACE_SECONDS) {
            clear();
        }
        if (!Double.isFinite(lastOdometryTime)) {
            turretReadyTime = now + VisionLocalizationConstants.TURRET_INITIAL_SETTLE_SECONDS;
            status = "Waiting for qualifying vision frame";
        }
        if (pinpointMissing) {
            // Avoid interpolating a camera observation across missing odometry samples.
            earliestCaptureTime = now;
            pinpointMissing = false;
            resetRelocalization();
        }
        lastOdometryTime = now;
        history.add(now, odometry, turretClockwiseDegrees, turretTrackingErrorDegrees);
        wpi.updateWithTime(now, odometry);
        ekf.updateWithTime(now, odometry);

        if (frame == null) {
            return;
        }

        double capture = frame.approximateCaptureNanos / 1e9;
        PoseHistory.Sample past = history.sample(capture);
        if (past == null || capture < earliestCaptureTime) {
            reject("Capture outside Pinpoint history");
            return;
        }
        if (capture < turretReadyTime) {
            reject("Waiting for initial turret center");
            return;
        }
        PoseHistory.Sample earlier = history.sample(capture - VisionLocalizationConstants.TURRET_SAMPLE_SECONDS);
        if (earlier == null) {
            reject("Waiting for turret angle history");
            return;
        }
        double turretSpeed = Math.abs(past.turretDegrees - earlier.turretDegrees)
                / VisionLocalizationConstants.TURRET_SAMPLE_SECONDS;
        if (turretSpeed > VisionLocalizationConstants.maxTurretSpeedDegreesPerSecond
                || past.turretTrackingErrorDegrees > VisionLocalizationConstants.MAX_TURRET_TRACKING_ERROR_DEGREES) {
            reject("Turret motion too fast at exposure");
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
            clearHiveMotion();
            lastTilt = Double.NaN;
            reject("HIVE geometry / floor / level mismatch");
            return;
        }

        correctedVision = toPinpointFrame(result.chassisPose);
        if (!isWithinField(correctedVision)) {
            tilt = Double.NaN;
            clearHiveMotion();
            lastTilt = Double.NaN;
            resetRelocalization();
            reject("Corrected vision pose outside field bounds");
            return;
        }
        tilt = result.hiveTiltDegrees;
        updateHiveMotion(capture, frame.pipeline);
        if (frame.pipeline != lastPipeline || !Double.isFinite(lastTilt)
                || capture - lastFrameTime > VisionLocalizationConstants.MAX_FRAME_GAP_SECONDS
                || Math.abs(tilt - lastTilt) > VisionLocalizationConstants.MAX_TILT_STEP_DEGREES) {
            stableSince = capture;
            resetRelocalization();
        }
        lastPipeline = frame.pipeline;
        lastTilt = tilt;
        lastFrameTime = capture;
        if (capture - stableSince < VisionLocalizationConstants.HIVE_STABLE_SECONDS) {
            reject("Waiting for stable HIVE tilt");
            return;
        }

        double distance = frame.averageDistanceMeters;
        double xyStd = VisionLocalizationConstants.minimumVisionXyStdMeters
                + VisionLocalizationConstants.visionDistanceNoiseCoefficient * distance * distance
                / Math.sqrt(frame.tagCount);
        double headingStd = Math.toRadians(Math.hypot(VisionLocalizationConstants.visionHeadingStdDegrees,
                turretSpeed * VisionLocalizationConstants.TURRET_TIMING_UNCERTAINTY_SECONDS));
        if (!Double.isFinite(xyStd) || !Double.isFinite(headingStd)) {
            reject("Invalid measurement noise");
            return;
        }

        // Require agreement with raw odometry before the first calibration of this OpMode.
        if (!ekf.isSeeded() && (correctedVision.getTranslation().getDistance(past.odometry.getTranslation())
                > VisionLocalizationConstants.MAX_SEED_DIFFERENCE_METERS
                || Math.abs(correctedVision.getRotation().minus(past.odometry.getRotation()).getDegrees())
                > VisionLocalizationConstants.MAX_SEED_HEADING_ERROR_DEGREES)) {
            if (accepted == 0) {
                reject("Vision and Pinpoint disagree at initialization");
            } else {
                relocalize(past, xyStd, headingStd);
            }
            return;
        }

        boolean ekfAccepted = ekf.addVisionMeasurement(correctedVision, capture, xyStd,
                headingStd, past.odometry);
        if (!ekfAccepted && ekf.isSeeded() && !hasUsableVisionForAim()) {
            relocalize(past, xyStd, headingStd);
            return;
        }

        boolean wpiAccepted = wpi.addVisionMeasurement(correctedVision, capture, xyStd, headingStd);
        String resultStatus = "WPI " + (wpiAccepted ? "accepted" : "rejected") + " / EKF " + ekf.getStatus();
        if (ekfAccepted) {
            accept(capture, resultStatus);
        } else {
            reject(resultStatus);
        }
    }

    private void relocalize(PoseHistory.Sample sample, double xyStd, double headingStd) {
        if (confirmRelocalization(sample) && ekf.reseedFromVision(correctedVision, sample.time,
                xyStd, headingStd, sample.odometry)) {
            wpi.reseedFromVision(correctedVision, sample.time);
            accept(sample.time, "EKF relocalized");
        } else {
            reject("Relocalizing: " + relocalizationFrames + " / "
                    + VisionLocalizationConstants.RELOCALIZATION_FRAME_COUNT + " matching frames");
        }
    }

    private boolean confirmRelocalization(PoseHistory.Sample sample) {
        if (relocalizationSample != null && sample.time <= relocalizationSample.time) {
            return false;
        }
        if (relocalizationSample == null
                || sample.time - relocalizationSample.time > VisionLocalizationConstants.MAX_FRAME_GAP_SECONDS) {
            relocalizationFrames = 0;
        } else {
            // Compare observations at the same time using intervening Pinpoint motion.
            Pose2d predicted = relocalizationPose.plus(sample.odometry.minus(relocalizationSample.odometry));
            double positionError = predicted.getTranslation().getDistance(correctedVision.getTranslation());
            double headingError = Math.abs(predicted.getRotation().minus(correctedVision.getRotation()).getDegrees());
            if (positionError > VisionLocalizationConstants.RELOCALIZATION_POSITION_TOLERANCE_METERS
                    || headingError > VisionLocalizationConstants.RELOCALIZATION_HEADING_TOLERANCE_DEGREES) {
                relocalizationFrames = 0;
            }
        }
        relocalizationPose = correctedVision;
        relocalizationSample = sample;
        relocalizationFrames++;
        return relocalizationFrames >= VisionLocalizationConstants.RELOCALIZATION_FRAME_COUNT;
    }

    private void resetRelocalization() {
        relocalizationSample = null;
        relocalizationPose = null;
        relocalizationFrames = 0;
    }

    private void accept(double capture, String reason) {
        accepted++;
        lastAcceptedTime = capture;
        status = reason;
        resetRelocalization();
    }

    private static Pose2d toPinpointFrame(Pose2d mapPose) {
        Rotation2d axes = Rotation2d.fromDegrees(VisionLocalizationConstants.MAP_TO_PINPOINT_YAW_DEGREES);
        double center = DistanceUnit.INCH.toMeters(VisionLocalizationConstants.FIELD_CENTER_INCHES);
        Rotation2d heading = mapPose.getRotation().plus(axes);
        Translation2d position = mapPose.getTranslation().rotateBy(axes)
                .plus(new Translation2d(center, center));
        Translation2d pivotOffset = new Translation2d(VisionLocalizationConstants.TURRET_PIVOT_FORWARD_METERS,
                VisionLocalizationConstants.TURRET_PIVOT_LEFT_METERS);
        return new Pose2d(position.minus(pivotOffset.rotateBy(heading)), heading);
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
        if (capture <= lastFrameTime) {
            clearHiveMotion();
            return;
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
        status = reason;
        rejected++;
    }

    public boolean hasFieldPose() { return ekf.isSeeded(); }
    public boolean hasUsableVisionForAim() {
        return ekf.isSeeded() && !pinpointMissing
                && getLastVisionAgeSeconds() <= VisionLocalizationConstants.MAX_VISION_AGE_FOR_AIM_SECONDS;
    }
    public Pose2d getEstimatedPosition() { return ekf.getEstimatedPosition(); }
    public Pose2d getWpiPosition() { return wpi.getEstimatedPosition(); }
    public Pose2d getCorrectedVision() { return correctedVision; }
    /** Latest valid tilt, in degrees, or NaN when the observation is stale. */
    public double getHiveTiltDegrees() {
        return lastUpdateTime - lastFrameTime
                <= VisionLocalizationConstants.MAX_FRAME_GAP_SECONDS ? tilt : Double.NaN;
    }

    public HiveMotionState getHiveMotionState() {
        return Double.isFinite(getHiveTiltDegrees()) ? hiveMotionState : HiveMotionState.UNKNOWN;
    }
    public double getLastVisionAgeSeconds() { return lastUpdateTime - lastAcceptedTime; }
    public String getStatus() { return status; }
    public int getAcceptedCount() { return accepted; }
    public int getRejectedCount() { return rejected; }
}
