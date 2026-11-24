package org.firstinspires.ftc.teamcode.subsystems.localization;

public final class VisionLocalizationConstants {
    private VisionLocalizationConstants() { }

    // Convert the field-centered map into Killer King's Pinpoint axes.
    // Initial assumption: Killer King's +X points toward blue HIVE.
    public static final double MAP_TO_PINPOINT_YAW_DEGREES = -90;
    public static final double FIELD_CENTER_INCHES = 72;
    public static final double FIELD_BOUNDARY_MARGIN_METERS = 0.25;
    // Limelight's web camera pose uses the floor below the turret pivot.
    public static final double TURRET_PIVOT_FORWARD_METERS = 0;
    public static final double TURRET_PIVOT_LEFT_METERS = 0;

    public static final double TURRET_SAMPLE_SECONDS = 0.020;
    public static final double TURRET_INITIAL_SETTLE_SECONDS = 0.45;
    public static final double MAX_TURRET_TRACKING_ERROR_DEGREES = 5.0;
    public static double maxTurretSpeedDegreesPerSecond = 90.0;
    public static final double TURRET_TIMING_UNCERTAINTY_SECONDS = 0.04;
    public static final double HIVE_STABLE_SECONDS = 0.15;
    public static final double MAX_TILT_STEP_DEGREES = 2.0;
    public static final double MAX_FRAME_GAP_SECONDS = 0.2;

    public static final double MAX_SEED_DIFFERENCE_METERS = 0.75;
    public static final double MAX_SEED_HEADING_ERROR_DEGREES = 45;
    public static final double PINPOINT_DROPOUT_GRACE_SECONDS = 0.25;
    public static final double VISION_COAST_AFTER_SECONDS = 0.35;
    public static final double MAX_VISION_AGE_FOR_AIM_SECONDS = 1.0;
    public static final int RELOCALIZATION_FRAME_COUNT = 3;
    public static final double RELOCALIZATION_POSITION_TOLERANCE_METERS = 0.15;
    public static final double RELOCALIZATION_HEADING_TOLERANCE_DEGREES = 8.0;
    public static double minimumVisionXyStdMeters = 0.08;
    public static double visionDistanceNoiseCoefficient = 0.04;
    public static double visionHeadingStdDegrees = 20.0;
}
