package org.firstinspires.ftc.teamcode.subsystems.localization;

public final class VisionLocalizationConstants {
    private VisionLocalizationConstants() { }

    // Convert the field-centered map into Killer King's Pinpoint axes.
    // The red alliance's audience-side corner is (0, 0). +X points toward blue.
    // In the centered Limelight map, +Y points toward blue and +X toward the audience.
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
    public static final double MAX_FRAME_GAP_SECONDS = 0.2;

    public static final double MIN_ALIGNMENT_POSITION_ERROR_METERS = 0.30;
    public static final double ALIGNMENT_CONFIRMATION_SECONDS = 0.60;
    public static final int MIN_ALIGNMENT_FRAMES = 3;
    public static final double ALIGNMENT_POSITION_TOLERANCE_METERS = 0.12;
}
