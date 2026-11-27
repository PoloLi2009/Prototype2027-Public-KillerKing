package org.firstinspires.ftc.teamcode.subsystems.localization;

import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

public final class VisionLocalizationConstants {
    private VisionLocalizationConstants() { }

    // Limelight's "robot" is the turret, and its yaw is the camera (launcher) direction: with the
    // turret centered that faces the chassis' right, -90 deg from the chassis heading. The vision
    // chassis heading subtracts this, so the Limelight web camera yaw stays 0. Without it the vision
    // heading read 90 deg less than Pinpoint (0 deg pointed to the scoring / referee side).
    // Confirmed 2026-11-27: the Limelight web camera yaw is 0. If anyone changes it, subtract the new
    // web yaw here as well (vision heading = camera yaw + web yaw - turret angle - this).
    public static final double TURRET_CENTER_CAMERA_YAW_DEGREES = TurretConstants.CENTER_FACING_DEGREES;

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
    // A forced alignment (TeleOp circle / B) is dropped if no valid frame arrives within this time.
    public static final double FORCED_ALIGNMENT_TIMEOUT_SECONDS = 1.0;

    // Forced heading: after the forced X/Y, average this many frames taken with the turret and the
    // chassis still. The vision heading adds the turret angle, which is only estimated from the
    // servo command (no feedback), so moving frames are not used. Without enough still frames within
    // the timeout (counted from the button press) only the X/Y correction stays.
    public static final int FORCED_HEADING_FRAMES = 5;
    public static final double FORCED_HEADING_TIMEOUT_SECONDS = 2.0;
    // Still = the turret estimate has reached its command and, over the last STILL_SECONDS, moved at
    // most MAX_TURRET_CHANGE while the chassis turned at most MAX_CHASSIS_TURN.
    public static final double FORCED_HEADING_STILL_SECONDS = 0.25;
    public static final double FORCED_HEADING_MAX_TURRET_ERROR_DEGREES = 0.5;
    public static final double FORCED_HEADING_MAX_TURRET_CHANGE_DEGREES = 0.5;
    public static final double FORCED_HEADING_MAX_CHASSIS_TURN_DEGREES = 1.0;
    // If any frame is farther than this from the average, the frames are discarded and re-collected.
    public static final double FORCED_HEADING_MAX_SPREAD_DEGREES = 2.0;
}
