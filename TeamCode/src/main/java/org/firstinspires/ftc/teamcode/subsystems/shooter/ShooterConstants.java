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

    // Plain PIDF, nothing else (user, 2026-11-27: no voltage compensation, no KS term, no gated integral):
    // power = KF * target + KP * error + KI * integral + KD * derivative, clipped to MIN / MAX_POWER.
    // Shooter Auto Tuner, 2026-11-27 (13.42 -> 13.09 V, 20 ms loop): steady RPM = 6404 * power - 378,
    // time constant 1.12 s; KF 0.000174 fits 3275-3600 RPM at 13.3 V. KF 0.00018 is the same at about
    // 12.8 V, a usual match battery: without voltage compensation the whole volley moves with the
    // battery, in the volley simulation about +15 RPM at 13.3 V and -19 RPM at 12.2 V (trim with the
    // D-pad). KP 0.0014: the tuner's best dip recovery; 0.002 and up push balls 2-4 above ball 1.
    // KI 0: any KI winds up on every ball's dip, so each ball of a volley comes out higher than the one
    // before (simulation: KI 0.0003 / 0.0007 put ball 4 15 / 30 RPM above ball 1). KD 0: it only
    // amplifies the +/-18 RPM encoder noise.
    public static final double KP = 0.0014;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KF = 0.00018;
    public static final double MANUAL_POWER_STEP = 0.01;
    public static final double INITIAL_MANUAL_POWER = KF * INITIAL_RPM;
    public static final double MIN_POWER = -0.1;
    public static final double MAX_POWER = 1.0;
    // First-order low-pass on the measured RPM, by time constant: each loop moves the filtered RPM by
    // dt / (FILTER_TIME_CONSTANT_SECONDS + dt) of the way to the raw reading, so the lag is the same at any
    // loop time. Longer smooths the readout but makes volleys worse (each ball's dip is seen later).
    public static final double FILTER_TIME_CONSTANT_SECONDS = 0.03;
    public static final double MIN_CONTROL_DT_SECONDS = 1e-4;

    // TeleOp volley start (FeedCommand): after RT opens the gate, the intake and channel motors start
    // once the filtered RPM is within FEED_START_RPM of the target, or after FEED_MAX_WAIT_SECONDS
    // anyway (a flywheel that cannot reach the target, or a bad encoder, must not block shooting).
    // Once started they run until RT is released and never pause for the flywheel (user, 2026-11-27;
    // the earlier mid-volley pauses, 80 / 35 RPM and then 120 / 70 RPM, were removed).
    public static final double FEED_START_RPM = 70;
    public static final double FEED_MAX_WAIT_SECONDS = 0.3;

    // Shot log (diagnostics only, does not change the control): a ball is counted when the filtered RPM
    // falls SHOT_DROP_RPM below its highest value of the last SHOT_WINDOW_SECONDS while under the target
    // by at least SHOT_BELOW_TARGET_RPM; it ends once the RPM is SHOT_REARM_RPM above the lowest point.
    // Shots more than VOLLEY_GAP_SECONDS apart start a new volley. Telemetry shows the last volley and
    // every shot goes to RobotLog (tag "Shooter").
    public static final double SHOT_DROP_RPM = 60;
    public static final double SHOT_BELOW_TARGET_RPM = 30;
    public static final double SHOT_WINDOW_SECONDS = 0.15;
    public static final double SHOT_REARM_RPM = 25;
    public static final double VOLLEY_GAP_SECONDS = 0.8;
    public static final int SHOTS_SHOWN = 6;

    // Distance to the target Hive (cm) and flywheel RPM, measured 2026-11-27 with KP 0.0014 / KI 0.0007 /
    // KF 0.000174, where the flywheel held its target. The +50 added later the same day only made up for
    // the steady 80-100 RPM shortfall of KI 0 with KF 0.000145, so the table is back to the measured
    // values. Check with test shots (the PIDF above holds the target at about 12.8 V).
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
        F(KF, 0.000002);

        public final double initialValue;
        public final double step;

        Gain(double initialValue, double step) {
            this.initialValue = initialValue;
            this.step = step;
        }
    }
}
