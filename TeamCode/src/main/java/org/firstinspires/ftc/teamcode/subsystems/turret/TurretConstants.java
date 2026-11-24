package org.firstinspires.ftc.teamcode.subsystems.turret;

public final class TurretConstants {
    private TurretConstants() { }

    public static final String FIRST_SERVO_NAME = "s2";
    public static final String SECOND_SERVO_NAME = "s01";
    public static final double CENTER_POSITION = 0.5;
    public static final double SERVO_TRAVEL_DEGREES = 350.0;
    // Initial slew estimate for camera compensation. Tune from loaded turret travel.
    public static final double ESTIMATED_SPEED_DEGREES_PER_SECOND = 300.0;
    public static final double MAX_OFFSET_DEGREES = 135.0;
    public static final double CENTER_FACING_DEGREES = 180.0;

    // Field coordinates used by the turret's aim command.
    public static final double BLUE_TARGET_X_INCHES = 84.0;
    // The red HIVE is mirrored across field X = 72 in in the shared Pinpoint frame.
    public static final double RED_TARGET_X_INCHES = 60.0;
    public static final double LOW_TARGET_Y_INCHES = 58.0;
    public static final double HIGH_TARGET_Y_INCHES = 96.0;
    public static final double SWITCH_Y_INCHES = 72.0;
}
