package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.qualcomm.robotcore.hardware.DcMotor;

public final class ShooterConstants {
    private ShooterConstants() { }

    public static final String ENCODER_MOTOR_NAME = "00";
    public static final String FOLLOWER_MOTOR_NAME = "01";
    public static final DcMotor.RunMode MOTOR_RUN_MODE = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;
    public static final DcMotor.Direction ENCODER_MOTOR_DIRECTION = DcMotor.Direction.FORWARD;

    public static final double TICKS_PER_REVOLUTION = 28.0;
    public static final double INITIAL_RPM = 3375;
    public static final double MIN_RPM = 0.0;
    public static final double MAX_RPM = 6000;
    public static final double RPM_STEP = 25;
    public static final double KP = 0.0050;
    public static final double KI = 0.0;
    public static final double KD = 0.0000;
    public static final double KF = 0.00020;
    public static final double MANUAL_POWER_STEP = 0.01;
    public static final double INITIAL_MANUAL_POWER = KF * INITIAL_RPM;
    public static final double MIN_POWER = -0.1;
    public static final double MAX_POWER = 1.0;
    public static final double FILTER_ALPHA = 0.4;
    public static final double MIN_CONTROL_DT_SECONDS = 1e-4;

    // Measured distance to the target Hive (cm) and corresponding flywheel RPM.
    static final double[][] DISTANCE_RPM_POINTS = {
            {109.4, 3212},
            {125.2, 3312},
            {143.0, 3462}
    };

    public enum Gain {
        P(KP, 0.00010),
        I(KI, 0.00001),
        D(KD, 0.000001),
        F(KF, 0.00001);

        public final double initialValue;
        public final double step;

        Gain(double initialValue, double step) {
            this.initialValue = initialValue;
            this.step = step;
        }
    }
}
