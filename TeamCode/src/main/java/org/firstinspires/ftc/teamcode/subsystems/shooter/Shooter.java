package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants.Gain;

import java.util.EnumMap;

public final class Shooter extends SubsystemBase {
    private final DcMotorEx encoderMotor;
    private final DcMotorEx followerMotor;
    private final EnumMap<Gain, Double> gains = new EnumMap<>(Gain.class);
    private Gain selectedGain = Gain.P;
    private double targetRpm = ShooterConstants.INITIAL_RPM;
    private double fixedRpm = ShooterConstants.INITIAL_RPM;
    private double rpmTrim;
    private boolean distanceRpmEnabled;
    private double filteredRpm;
    private double lastRawRpm;
    private double lastTime;
    private double power;
    private double powerOverride = Double.NaN;
    private double integralPower;
    private boolean hasPreviousFeedbackSample;
    private boolean manual;
    private double manualPower = ShooterConstants.INITIAL_MANUAL_POWER;

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
    }

    public void startControl(double now) {
        lastTime = now;
        resetFeedback();
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
        filteredRpm += ShooterConstants.FILTER_ALPHA * (rawRpm - filteredRpm);
        if (manual) {
            setPower(manualPower);
            return;
        }
        double error = targetRpm - filteredRpm;
        // Differentiate the measured RPM so a setpoint step does not kick the output.
        double derivative = hasPreviousFeedbackSample ? (previousRpm - filteredRpm) / dt : 0;
        hasPreviousFeedbackSample = true;
        double basePower = gains.get(Gain.F) * targetRpm
                + gains.get(Gain.P) * error + gains.get(Gain.D) * derivative;
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
        telemetry.addData("Shooter 实际 RPM", "%.0f", filteredRpm);
        telemetry.addData("Shooter 功率", "%.3f", power);
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
