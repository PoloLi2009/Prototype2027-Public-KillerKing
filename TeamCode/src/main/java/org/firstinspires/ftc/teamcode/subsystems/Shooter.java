package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class Shooter extends SubsystemBase {
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM = 6000;
    private static final double INITIAL_RPM = 3375;
    private static final double RPM_STEP = 25;
    private static final double KP = 0.0030;
    private static final double KD = 0.0000;
    private static final double KF = 0.000173;
    private static final double MIN_POWER = -0.1;
    private static final double MAX_POWER = 1.0;
    private static final double FILTER_ALPHA = 0.4;

    private final DcMotorEx encoderMotor;
    private final DcMotorEx followerMotor;
    private double targetRpm = INITIAL_RPM;
    private double filteredRpm;
    private double lastError;
    private double lastTime;
    private double power;

    public Shooter(HardwareMap hardwareMap) {
        encoderMotor = hardwareMap.get(DcMotorEx.class, "00");
        followerMotor = hardwareMap.get(DcMotorEx.class, "01");
        for (DcMotorEx motor : new DcMotorEx[]{encoderMotor, followerMotor}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        encoderMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void startControl(double now) {
        lastTime = now;
        lastError = 0;
    }

    public void increaseRpm() {
        targetRpm = Range.clip(targetRpm + RPM_STEP, 0, MAX_RPM);
    }

    public void decreaseRpm() {
        targetRpm = Range.clip(targetRpm - RPM_STEP, 0, MAX_RPM);
    }

    public void updateControl(double now) {
        double dt = Math.max(now - lastTime, 1e-4);
        lastTime = now;

        double rawRpm = -encoderMotor.getVelocity() * 60.0 / TICKS_PER_REV;
        filteredRpm += FILTER_ALPHA * (rawRpm - filteredRpm);
        double error = targetRpm - filteredRpm;
        double derivative = (error - lastError) / dt;
        lastError = error;
        setPower(Range.clip(KF * targetRpm + KP * error + KD * derivative, MIN_POWER, MAX_POWER));
    }

    public void stop() {
        setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter 目标 RPM", "%.0f", targetRpm);
        telemetry.addData("Shooter 实际 RPM", "%.0f", filteredRpm);
        telemetry.addData("Shooter 功率", "%.3f", power);
    }

    private void setPower(double power) {
        this.power = power;
        encoderMotor.setPower(-power);
        followerMotor.setPower(power);
    }
}
