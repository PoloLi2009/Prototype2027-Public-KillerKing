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
    // 70 RPM fast there). KP 0.0014 had the best dip recovery without overshoot; KD stays 0 because
    // it only amplifies the +/-18 RPM encoder noise. KI is 0 (user, 2026-11-27; it was 0.0007): the
    // integral grew on every ball's dip and pushed the flywheel over for the next ball. The battery
    // drop it made up for is now handled by voltage compensation (below). Before (KP 0.001, KI 0,
    // KD 0.0001, KF 0.0002) the auto RPMs were set.
    public static final double KP = 0.001;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KF = 0.000145;
    // Voltage compensation: the PIDF output is multiplied by NOMINAL_VOLTAGE / battery voltage, so the
    // flywheel gets the same motor voltage as at NOMINAL_VOLTAGE whatever the battery does. KF and KP
    // above are the values at NOMINAL_VOLTAGE (the tuner ran at 13.4 V); at 13.4 V nothing changes.
    // Manual power, open-loop power and the tuner's power override are sent as they are.
    public static final double NOMINAL_VOLTAGE = 13.4;
    // The battery is read this often (each read is a separate hub transaction), low-pass filtered by
    // VOLTAGE_FILTER_ALPHA per read. Readings outside MIN / MAX_VALID_VOLTAGE are ignored.
    public static final double VOLTAGE_READ_PERIOD_SECONDS = 0.1;
    public static final double VOLTAGE_FILTER_ALPHA = 0.5;
    public static final double MIN_VALID_VOLTAGE = 7.0;
    public static final double MAX_VALID_VOLTAGE = 16.0;
    public static final double MANUAL_POWER_STEP = 0.01;
    public static final double INITIAL_MANUAL_POWER = KF * INITIAL_RPM;
    public static final double MIN_POWER = -0.1;
    public static final double MAX_POWER = 1.0;
    // First-order low-pass on the measured RPM, by time constant: each loop moves the filtered RPM by
    // dt / (FILTER_TIME_CONSTANT_SECONDS + dt) of the way to the raw reading, so the lag is the same at any
    // loop time. 30 ms equals the old fixed 0.4 per loop at the tuner's 20 ms loop (the gains were tuned
    // with it); at the autos' 30-40 ms loops 0.4 meant 45-60 ms of lag. Longer smooths the readout but
    // makes volleys worse (each ball's dip is seen later); see the README.
    public static final double FILTER_TIME_CONSTANT_SECONDS = 0.03;
    public static final double MIN_CONTROL_DT_SECONDS = 1e-4;

    // Distance to the target Hive (cm) and flywheel RPM, measured 2026-11-27 with KP 0.0014 / KI 0.0007 /
    // KF 0.000174 and no voltage compensation (3275 / 3350 / 3400 / 3600), then every point raised by
    // 50 RPM the same day (user) after the switch to KI 0 and voltage compensation. Check with test shots.
    // (Before, under KP 0.001 / KD 0.0001 / KF 0.0002: 109.4 / 3177, 125.2 / 3277, 143.0 / 3427.)
    static final double[][] DISTANCE_RPM_POINTS = {
            {104.0, 3325},
            {118.4, 3400},
            {138.4, 3450},
            {170.0, 3650}
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
