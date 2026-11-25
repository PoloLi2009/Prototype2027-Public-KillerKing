package org.firstinspires.ftc.teamcode.subsystems.shooter;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public final class Shooter extends SubsystemBase {
    private final DcMotorEx encoderMotor;
    private final DcMotorEx followerMotor;
    private double targetRpm = ShooterConstants.INITIAL_RPM;
    private double filteredRpm;
    private double lastError;
    private double lastTime;
    private double power;

    public Shooter(HardwareMap hardwareMap) {
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
        lastError = 0;
    }

    public void increaseRpm() {
        targetRpm = Range.clip(targetRpm + ShooterConstants.RPM_STEP,
                ShooterConstants.MIN_RPM, ShooterConstants.MAX_RPM);
    }

    public void decreaseRpm() {
        targetRpm = Range.clip(targetRpm - ShooterConstants.RPM_STEP,
                ShooterConstants.MIN_RPM, ShooterConstants.MAX_RPM);
    }

    public void updateControl(double now) {
        double dt = Math.max(now - lastTime, ShooterConstants.MIN_CONTROL_DT_SECONDS);
        lastTime = now;

        double rawRpm = encoderMotor.getVelocity() * 60.0 / ShooterConstants.TICKS_PER_REVOLUTION;
        filteredRpm += ShooterConstants.FILTER_ALPHA * (rawRpm - filteredRpm);
        double error = targetRpm - filteredRpm;
        double derivative = (error - lastError) / dt;
        lastError = error;
        double requestedPower = ShooterConstants.KF * targetRpm
                + ShooterConstants.KP * error + ShooterConstants.KD * derivative;
        setPower(Range.clip(requestedPower, ShooterConstants.MIN_POWER, ShooterConstants.MAX_POWER));
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
