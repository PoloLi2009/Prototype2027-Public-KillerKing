package org.firstinspires.ftc.teamcode.subsystems.localization;

public final class VisionLocalizationConstants {
    private VisionLocalizationConstants() { }

    // Convert the field-centered map into Killer King's Pinpoint axes.
    // Initial assumption: Killer King's +X points toward blue HIVE.
    public static final double MAP_TO_PINPOINT_YAW_DEGREES = -90;
    public static final double FIELD_CENTER_INCHES = 72;
    // Limelight's web camera pose uses the floor below the turret pivot.
    public static final double TURRET_PIVOT_FORWARD_METERS = 0;
    public static final double TURRET_PIVOT_LEFT_METERS = 0;

    public static final double TURRET_SAMPLE_SECONDS = 0.020;
    public static final double TURRET_ANGLE_STEP_DEGREES = 1.0;
    public static final double TURRET_SETTLE_SECONDS = 0.25;
    public static double maxTurretSpeedDegreesPerSecond = 180.0;
    public static final double HIVE_STABLE_SECONDS = 0.15;
    public static final double MAX_TILT_STEP_DEGREES = 2.0;
    public static final double MAX_FRAME_GAP_SECONDS = 0.2;

    public static final double MAX_SEED_DIFFERENCE_METERS = 0.75;
    public static final double MAX_SEED_HEADING_ERROR_DEGREES = 45;
    public static double minimumVisionXyStdMeters = 0.08;
    public static double visionDistanceNoiseCoefficient = 0.04;
    public static double visionHeadingStdDegrees = 20.0;
}
