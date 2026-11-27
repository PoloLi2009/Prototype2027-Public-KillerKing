package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.Gain;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Locale;

public final class Shooter extends SubsystemBase {
    private final DcMotorEx encoderMotor;
    private final DcMotorEx followerMotor;
    private final List<VoltageSensor> voltageSensors;   // only read for the shot log, not for control
    private final EnumMap<Gain, Double> gains = new EnumMap<>(Gain.class);
    private Gain selectedGain = Gain.P;
    private double targetRpm = ShooterConstants.INITIAL_RPM;
    private double fixedRpm = ShooterConstants.INITIAL_RPM;
    private double rpmTrim;
    private boolean distanceRpmEnabled;
    private double filteredRpm;
    private boolean filterPrimed;                 // false until the first update after startControl()
    private double lastRawRpm;
    private double lastTime;
    private double power;
    private double powerOverride = Double.NaN;
    private double integralPower;
    private boolean hasPreviousFeedbackSample;
    private boolean readyToFeed;                  // |error| <= FEED_START_RPM (always in manual mode)
    private boolean manual;
    private double manualPower = ShooterConstants.INITIAL_MANUAL_POWER;

    // Shot log: filtered RPM of the last SHOT_WINDOW_SECONDS, the shot in progress and the last volley.
    private static final String LOG_TAG = "Shooter";
    private static final int SHOT_WINDOW_SIZE = 32;
    private final double[] windowTime = new double[SHOT_WINDOW_SIZE];
    private final double[] windowRpm = new double[SHOT_WINDOW_SIZE];
    private int windowCount;
    private int windowNext;
    private boolean inShot;
    private double shotStartTime;
    private double shotTargetRpm;
    private double shotBeforeRpm;
    private double shotLowestRpm;
    private double lastShotTime = Double.NEGATIVE_INFINITY;
    private int volleyNumber;
    private int shotInVolley;
    private final List<Shot> volley = new ArrayList<>();

    private static final class Shot {
        final double targetRpm, beforeRpm, dipRpm, sincePreviousS;

        Shot(double targetRpm, double beforeRpm, double dipRpm, double sincePreviousS) {
            this.targetRpm = targetRpm;
            this.beforeRpm = beforeRpm;
            this.dipRpm = dipRpm;
            this.sincePreviousS = sincePreviousS;
        }
    }

    public Shooter(HardwareMap hardwareMap) {
        for (Gain gain : Gain.values()) {
            gains.put(gain, gain.initialValue);
        }
        encoderMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.ENCODER_MOTOR_NAME);
        followerMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.FOLLOWER_MOTOR_NAME);
        for (DcMotorEx motor : new DcMotorEx[]{encoderMotor, followerMotor}) {
            motor.setMode(ShooterConstants.MOTOR_RUN_MODE);
            motor.setZeroPowerBehavior(ShooterConstants.ZERO_POWER_BEHAVIOR);
        }
        encoderMotor.setDirection(ShooterConstants.ENCODER_MOTOR_DIRECTION);
        voltageSensors = hardwareMap.getAll(VoltageSensor.class);
    }

    public void startControl(double now) {
        lastTime = now;
        resetFeedback();
        filterPrimed = false;                             // start the filter at the first reading
        readyToFeed = false;
        inShot = false;
        windowCount = 0;
    }

    public void adjustRpm(int direction) {
        double change = direction * ShooterConstants.RPM_STEP;
        if (distanceRpmEnabled) {
            double adjustedRpm = Range.clip(targetRpm + change, ShooterConstants.MIN_RPM, ShooterConstants.MAX_RPM);
            rpmTrim += adjustedRpm - targetRpm;
            targetRpm = adjustedRpm;
        } else {
            fixedRpm = Range.clip(fixedRpm + change, ShooterConstants.MIN_RPM, ShooterConstants.MAX_RPM);
        }
    }

    public void toggleDistanceRpm() {
        distanceRpmEnabled = !distanceRpmEnabled;
        resetFeedback();
    }

    /**
     * Sets the fixed-mode flywheel target (used by autonomous); the TeleOp keeps using the D-pad
     * steps. {@link #updateControl} copies it into the target while distance RPM is off.
     */
    public void setTargetRpm(double rpm) {
        fixedRpm = Range.clip(rpm, ShooterConstants.MIN_RPM, ShooterConstants.MAX_RPM);
        if (!distanceRpmEnabled) {
            targetRpm = fixedRpm;
        }
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getFilteredRpm() {
        return filteredRpm;
    }

    public void updateControl(double now, double hiveDistanceCm) {
        if (distanceRpmEnabled) {
            double baseRpm = Double.isFinite(hiveDistanceCm) ? interpolateRpm(hiveDistanceCm) : fixedRpm;
            targetRpm = Range.clip(baseRpm + rpmTrim, ShooterConstants.MIN_RPM, ShooterConstants.MAX_RPM);
        } else {
            targetRpm = fixedRpm;
        }

        double dt = Math.max(now - lastTime, ShooterConstants.MIN_CONTROL_DT_SECONDS);
        lastTime = now;

        double rawRpm = readRawRpm();
        lastRawRpm = rawRpm;
        double previousRpm = filteredRpm;
        if (filterPrimed) {
            // First-order low-pass by time constant: the same lag whatever the loop time.
            filteredRpm += dt / (ShooterConstants.FILTER_TIME_CONSTANT_SECONDS + dt) * (rawRpm - filteredRpm);
        } else {
            // A stale value (from before a pause, or 0 at the first start) would make P see a false
            // error for the first loops; resetFeedback() already makes the first derivative 0.
            filteredRpm = rawRpm;
            filterPrimed = true;
        }
        double error = targetRpm - filteredRpm;
        updateReadyToFeed(error);
        detectShot(now);
        if (manual) {
            setPower(manualPower);
            return;
        }
        // Plain PIDF. Differentiate the measured RPM so a setpoint step does not kick the output.
        double derivative = hasPreviousFeedbackSample ? (previousRpm - filteredRpm) / dt : 0;
        hasPreviousFeedbackSample = true;
        double basePower = gains.get(Gain.F) * targetRpm
                + gains.get(Gain.P) * error + gains.get(Gain.D) * derivative;
        // The integral only grows while the output has room left (anti-windup).
        double integralStep = gains.get(Gain.I) * error * dt;
        double requestedPower = basePower + integralPower;
        if (integralStep > 0 && requestedPower < ShooterConstants.MAX_POWER) {
            integralPower += Math.min(integralStep, ShooterConstants.MAX_POWER - requestedPower);
        } else if (integralStep < 0 && requestedPower > ShooterConstants.MIN_POWER) {
            integralPower += Math.max(integralStep, ShooterConstants.MIN_POWER - requestedPower);
        }
        double outputRange = ShooterConstants.MAX_POWER - ShooterConstants.MIN_POWER;
        integralPower = Range.clip(integralPower, -outputRange, outputRange);
        double output = Range.clip(basePower + integralPower, ShooterConstants.MIN_POWER, ShooterConstants.MAX_POWER);
        setPower(Double.isNaN(powerOverride) ? output : powerOverride);
    }

    private void updateReadyToFeed(double error) {
        readyToFeed = manual || Math.abs(error) <= ShooterConstants.FEED_START_RPM;
    }

    /**
     * True while the filtered RPM is within FEED_START_RPM of the target: the TeleOp starts a volley
     * only then (it does not pause once started). Always true in manual power mode; false until the
     * first updateControl() after startControl().
     */
    public boolean isReadyToFeed() {
        return readyToFeed;
    }

    // ---------------------------------------------------------------- shot log

    private void detectShot(double now) {
        if (inShot) {
            shotLowestRpm = Math.min(shotLowestRpm, filteredRpm);
            if (filteredRpm < shotLowestRpm + ShooterConstants.SHOT_REARM_RPM) return;
            finishShot();
            windowCount = 0;                      // the RPM before this shot must not start another
        }
        windowTime[windowNext] = now;
        windowRpm[windowNext] = filteredRpm;
        windowNext = (windowNext + 1) % SHOT_WINDOW_SIZE;
        windowCount = Math.min(windowCount + 1, SHOT_WINDOW_SIZE);
        double highest = Double.NEGATIVE_INFINITY;
        for (int k = 1; k <= windowCount; k++) {
            int i = Math.floorMod(windowNext - k, SHOT_WINDOW_SIZE);
            if (now - windowTime[i] > ShooterConstants.SHOT_WINDOW_SECONDS) break;
            highest = Math.max(highest, windowRpm[i]);
        }
        if (targetRpm > 0 && highest - filteredRpm >= ShooterConstants.SHOT_DROP_RPM
                && targetRpm - filteredRpm >= ShooterConstants.SHOT_BELOW_TARGET_RPM) {
            inShot = true;
            shotStartTime = now;
            shotTargetRpm = targetRpm;
            shotBeforeRpm = highest;
            shotLowestRpm = filteredRpm;
        }
    }

    private void finishShot() {
        inShot = false;
        double sincePrevious = shotStartTime - lastShotTime;
        if (!(sincePrevious <= ShooterConstants.VOLLEY_GAP_SECONDS)) {
            volley.clear();
            volleyNumber++;
            shotInVolley = 0;
            sincePrevious = Double.NaN;
        }
        lastShotTime = shotStartTime;
        shotInVolley++;
        Shot shot = new Shot(shotTargetRpm, shotBeforeRpm, shotBeforeRpm - shotLowestRpm, sincePrevious);
        if (volley.size() >= ShooterConstants.SHOTS_SHOWN) volley.remove(0);
        volley.add(shot);
        RobotLog.ii(LOG_TAG, "volley %d shot %d: target %.0f, before %.0f (%+.0f), dip %.0f, %s, %.2f V, I %+.3f",
                volleyNumber, shotInVolley, shot.targetRpm, shot.beforeRpm, shot.beforeRpm - shot.targetRpm,
                shot.dipRpm, Double.isNaN(sincePrevious) ? "first"
                        : String.format(Locale.US, "%.0f ms after previous", sincePrevious * 1000),
                readBatteryVoltage(), integralPower);
    }

    /** The lowest valid hub battery reading, NaN without one. Only the shot log reads it. */
    private double readBatteryVoltage() {
        double lowest = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltageSensors) {
            double volts = sensor.getVoltage();
            if (volts > 0) lowest = Math.min(lowest, volts);
        }
        return Double.isInfinite(lowest) ? Double.NaN : lowest;
    }

    /** Reads the flywheel encoder now. updateControl() reads it itself every loop. */
    public double readRawRpm() {
        return encoderMotor.getVelocity() * 60.0 / ShooterConstants.TICKS_PER_REVOLUTION;
    }

    /** The unfiltered RPM read by the last updateControl(). */
    public double getLastRawRpm() {
        return lastRawRpm;
    }

    /** The power sent to the flywheel last. */
    public double getPower() {
        return power;
    }

    /** Open-loop flywheel power for tuning. The next updateControl() sends its own output again. */
    public void setOpenLoopPower(double power) {
        setPower(Range.clip(power, ShooterConstants.MIN_POWER, ShooterConstants.MAX_POWER));
    }

    /**
     * Makes updateControl() send {@code power} instead of its output while it keeps filtering and
     * integrating (the tuner brakes the flywheel this way, like a ball taking energy). NaN clears it.
     */
    public void setPowerOverride(double power) {
        powerOverride = Double.isNaN(power) ? Double.NaN
                : Range.clip(power, ShooterConstants.MIN_POWER, ShooterConstants.MAX_POWER);
    }

    /** Replaces all four gains (the TeleOp tuning keys step from these) and clears the integral. */
    public void setGains(double p, double i, double d, double f) {
        gains.put(Gain.P, p);
        gains.put(Gain.I, i);
        gains.put(Gain.D, d);
        gains.put(Gain.F, f);
        resetFeedback();
    }

    private static double interpolateRpm(double distanceCm) {
        double[][] points = ShooterConstants.DISTANCE_RPM_POINTS;
        if (distanceCm <= points[0][0]) {
            return points[0][1];
        }
        for (int i = 1; i < points.length; i++) {
            double[] lower = points[i - 1];
            double[] upper = points[i];
            if (distanceCm <= upper[0]) {
                double fraction = (distanceCm - lower[0]) / (upper[0] - lower[0]);
                return lower[1] + fraction * (upper[1] - lower[1]);
            }
        }
        return points[points.length - 1][1];
    }

    public void selectGain(int direction) {
        Gain[] gainOptions = Gain.values();
        selectedGain = gainOptions[Math.floorMod(selectedGain.ordinal() + direction, gainOptions.length)];
    }

    public void adjustSelectedGain(int direction) {
        double step = selectedGain.step;
        long ticks = Math.round(gains.get(selectedGain) / step);
        gains.put(selectedGain, Math.max(0, ticks + direction) * step);
        if (selectedGain == Gain.I) {
            integralPower = 0;
        }
    }

    public void toggleManualPower() {
        manual = !manual;
        resetFeedback();
    }

    public void adjustManualPower(int direction) {
        manualPower = Range.clip(manualPower + direction * ShooterConstants.MANUAL_POWER_STEP,
                0, ShooterConstants.MAX_POWER);
    }

    private void resetFeedback() {
        integralPower = 0;
        hasPreviousFeedbackSample = false;
    }

    public void stop() {
        setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter 目标 RPM", "%.0f", targetRpm);
        if (distanceRpmEnabled) {
            // The D-pad ← / → offset added to the distance table (0 = the table as measured).
            telemetry.addData("Shooter RPM 微调", "%+.0f", rpmTrim);
        }
        telemetry.addData("Shooter 实际 RPM", "%.0f", filteredRpm);
        telemetry.addData("Shooter 功率", "%.3f", power);
        telemetry.addData("Shooter 积分 / 可送球", "%+.3f / %s", integralPower, readyToFeed ? "是" : "等转速");
        addShotTelemetry(telemetry);
    }

    /** The last volley: each ball's RPM error just before it (filtered peak - target) and its dip. */
    private void addShotTelemetry(Telemetry telemetry) {
        if (volley.isEmpty()) {
            telemetry.addData("上一轮出手", "还没有检测到射球");
            return;
        }
        StringBuilder errors = new StringBuilder();
        StringBuilder gaps = new StringBuilder();
        for (Shot shot : volley) {
            errors.append(String.format(Locale.US, " %+.0f/%.0f", shot.beforeRpm - shot.targetRpm, shot.dipRpm));
            if (!Double.isNaN(shot.sincePreviousS)) {
                gaps.append(String.format(Locale.US, " %.2f", shot.sincePreviousS));
            }
        }
        telemetry.addData("上一轮出手 误差/掉速 RPM", "第 %d 轮%s", volleyNumber, errors);
        if (gaps.length() > 0) telemetry.addData("上一轮出手 球间隔 s", gaps.toString().trim());
    }

    public void addTuningTelemetry(Telemetry telemetry) {
        String rpmMode = distanceRpmEnabled ? "按距离插值" : "固定 RPM";
        telemetry.addData("飞轮模式", manual ? "手动功率" : "PIDF / " + rpmMode);
        telemetry.addData("手动设定功率", "%.3f", manualPower);
        telemetry.addData("当前选择", selectedGain.name());
        telemetry.addData("P / I / D / F", "%.6f / %.6f / %.6f / %.6f",
                gains.get(Gain.P), gains.get(Gain.I), gains.get(Gain.D), gains.get(Gain.F));
    }

    private void setPower(double power) {
        this.power = power;
        encoderMotor.setPower(-power);
        followerMotor.setPower(power);
    }
}
