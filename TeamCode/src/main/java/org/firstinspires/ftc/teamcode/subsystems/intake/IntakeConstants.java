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

    public static final double INITIAL_LIFT_POSITION = 0.45;
    public static final double LIFT_STEP = 0.1;
    public static final double LIFT_MIN = 0.2;
    public static final double LIFT_MAX = 0.8;

    // The same power magnitude is used for intake, firing, and reverse.
    public static final double TRIGGER_DEADZONE = 0.01;
    public static final double INITIAL_FEED_POWER = 1.0;
    public static final double FEED_POWER_STEP = 0.1;
    public static final double MIN_FEED_POWER = 0.0;
    public static final double MAX_FEED_POWER = 1.0;
}
