package org.firstinspires.ftc.teamcode.subsystems.turret;

public final class TurretConstants {
    private TurretConstants() { }

    public static final String FIRST_SERVO_NAME = "s2";
    public static final String SECOND_SERVO_NAME = "s01";
    public static final double CENTER_POSITION = 0.5;
    public static final double SERVO_TRAVEL_DEGREES = 320.0;
    public static final double MAX_OFFSET_DEGREES = SERVO_TRAVEL_DEGREES / 2.0;
    // Both servos decrease position when turning the turret counterclockwise.
    public static final double SERVO_DIRECTION = -1.0;
    // Initial slew estimate for camera compensation. Tune from loaded turret travel.
    public static final double ESTIMATED_SPEED_DEGREES_PER_SECOND = 300.0;
    // Chassis-relative CCW degrees. The launcher points to the right at servo center.
    public static final double CENTER_FACING_DEGREES = -90.0;

    // Field coordinates used by the turret's aim command.
    public static final double BLUE_TARGET_X_INCHES = 84.0;
    public static final double RED_TARGET_X_INCHES = 60.0;
    public static final double LOW_TARGET_Y_INCHES = 60.0;
    public static final double HIGH_TARGET_Y_INCHES = 84.0;
    public static final double SWITCH_Y_INCHES = 72.0;
}
