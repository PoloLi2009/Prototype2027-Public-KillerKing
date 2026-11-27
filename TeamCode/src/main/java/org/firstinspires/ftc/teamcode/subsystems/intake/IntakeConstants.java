package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;

public final class IntakeConstants {
    private IntakeConstants() { }

    public static final String INTAKE_MOTOR_NAME = "03";
    public static final String FEED_MOTOR_NAME = "02";
    public static final String LIFT_SERVO_NAME = "s0";
    public static final String MIRRORED_LIFT_SERVO_NAME = "s00";
    public static final DcMotor.RunMode MOTOR_RUN_MODE = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;
    public static final DcMotor.Direction FEED_MOTOR_DIRECTION = DcMotor.Direction.REVERSE;

    // The intake stays at this one height in every OpMode: no raised / pickup positions any more
    // (2026-11-27; before: raised 0.70, pickup 0.53). Larger = higher. The TeleOp D-pad can still
    // nudge it by LIFT_STEP.
    public static final double LIFT_POSITION = 0.56;
    public static final double LIFT_STEP = 0.1;
    public static final double LIFT_MIN = 0.2;
    public static final double LIFT_MAX = 0.8;

    // Motor power magnitudes for intake, firing and reverse. FEED_MOTOR_POWER is the feed / channel
    // motor in the TeleOp (intake, firing, reverse) and in autonomous pickups.
    public static final double INTAKE_MOTOR_POWER = 1.0;
    public static final double FEED_MOTOR_POWER = 0.8;
    // Intake and feed / channel motors while the TeleOp fires (RT held, gate open), 2026-11-27 (user).
    // Intake and reverse keep INTAKE_MOTOR_POWER / FEED_MOTOR_POWER.
    public static final double FIRE_INTAKE_MOTOR_POWER = 0.6;
    public static final double FIRE_FEED_MOTOR_POWER = 0.8;
    // Feed / channel motor while the stable autonomous routines fire a volley (slower than TeleOp).
    public static final double AUTO_FIRE_FEED_MOTOR_POWER = 0.6;
    public static final double TRIGGER_DEADZONE = 0.01;
}
