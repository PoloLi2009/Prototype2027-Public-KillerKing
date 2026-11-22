package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public final class Intake extends SubsystemBase {
    private static final double LIFT_STEP = 0.1;
    private static final double LIFT_MIN = 0.2;
    private static final double LIFT_MAX = 0.8;

    private final DcMotorEx intakeMotor;
    private final DcMotorEx feedMotor;
    private final Servo liftServo;
    private final Servo mirroredLiftServo;
    private double liftPosition = 0.5;
    private double power;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "03");
        feedMotor = hardwareMap.get(DcMotorEx.class, "02");
        for (DcMotorEx motor : new DcMotorEx[]{intakeMotor, feedMotor}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        feedMotor.setDirection(DcMotor.Direction.REVERSE);
        liftServo = hardwareMap.get(Servo.class, "s0");
        mirroredLiftServo = hardwareMap.get(Servo.class, "s00");
        setLiftPosition(liftPosition);
    }

    public void raiseLift() {
        setLiftPosition(liftPosition - LIFT_STEP);
    }

    public void lowerLift() {
        setLiftPosition(liftPosition + LIFT_STEP);
    }

    public double getLiftPosition() {
        return liftPosition;
    }

    public void setPower(double power) {
        this.power = power;
        intakeMotor.setPower(power);
        feedMotor.setPower(power);
    }

    public double getPower() {
        return power;
    }

    public void stop() {
        setPower(0);
    }

    private void setLiftPosition(double position) {
        liftPosition = Range.clip(position, LIFT_MIN, LIFT_MAX);
        liftServo.setPosition(liftPosition);
        mirroredLiftServo.setPosition(1.0 - liftPosition);
    }
}
