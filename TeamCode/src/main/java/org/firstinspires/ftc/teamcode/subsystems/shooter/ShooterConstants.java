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
    // Shooter Auto Tuner, 2026-11-27, 13.4 V, 20 ms loop: 6404 RPM per unit power, time constant
    // 1.12 s, so 3375 RPM needs 0.586 power. KF fits a fresh battery (the old 0.0002 ran about
    // 70 RPM fast there); KI makes up for the battery dropping. KP 0.0014 had the best dip recovery
    // without overshoot; KD stays 0 because it only amplifies the +/-18 RPM encoder noise. Before
    // (KP 0.001, KI 0, KD 0.0001, KF 0.0002) the distance table and auto RPMs were set.
    public static final double KP = 0.0014;
    public static final double KI = 0.0007;
    public static final double KD = 0.0;
    public static final double KF = 0.000174;
    public static final double MANUAL_POWER_STEP = 0.01;
    public static final double INITIAL_MANUAL_POWER = KF * INITIAL_RPM;
    public static final double MIN_POWER = -0.1;
    public static final double MAX_POWER = 1.0;
    public static final double FILTER_ALPHA = 0.4;
    public static final double MIN_CONTROL_DT_SECONDS = 1e-4;

    // Distance to the target Hive (cm) and flywheel RPM, measured 2026-11-27 with the tuned gains above.
    // (Before, under KP 0.001 / KD 0.0001 / KF 0.0002: 109.4 / 3177, 125.2 / 3277, 143.0 / 3427.)
    static final double[][] DISTANCE_RPM_POINTS = {
            {104.0, 3275},
            {118.4, 3350},
            {138.4, 3400},
            {170.0, 3600}
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
