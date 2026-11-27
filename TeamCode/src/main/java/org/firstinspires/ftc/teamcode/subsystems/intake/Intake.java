package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public final class Intake extends SubsystemBase {
    private final DcMotorEx intakeMotor;
    private final DcMotorEx feedMotor;
    private final Servo liftServo;
    private final Servo mirroredLiftServo;
    private double liftPosition = IntakeConstants.LIFT_POSITION;
    private double intakePower;
    private double feedPower;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, IntakeConstants.INTAKE_MOTOR_NAME);
        feedMotor = hardwareMap.get(DcMotorEx.class, IntakeConstants.FEED_MOTOR_NAME);
        for (DcMotorEx motor : new DcMotorEx[]{intakeMotor, feedMotor}) {
            motor.setMode(IntakeConstants.MOTOR_RUN_MODE);
            motor.setZeroPowerBehavior(IntakeConstants.ZERO_POWER_BEHAVIOR);
        }
        feedMotor.setDirection(IntakeConstants.FEED_MOTOR_DIRECTION);
        liftServo = hardwareMap.get(Servo.class, IntakeConstants.LIFT_SERVO_NAME);
        mirroredLiftServo = hardwareMap.get(Servo.class, IntakeConstants.MIRRORED_LIFT_SERVO_NAME);
        setLiftPosition(liftPosition);
    }

    public void raiseLift() {
        setLiftPosition(liftPosition + IntakeConstants.LIFT_STEP);
    }

    public void lowerLift() {
        setLiftPosition(liftPosition - IntakeConstants.LIFT_STEP);
    }

    public double getLiftPosition() {
        return liftPosition;
    }

    /** Sets the intake motor (03) and the feed / channel motor (02); positive moves balls inward. */
    public void setPowers(double intakePower, double feedPower) {
        this.intakePower = intakePower;
        this.feedPower = feedPower;
        intakeMotor.setPower(intakePower);
        feedMotor.setPower(feedPower);
    }

    public double getIntakePower() {
        return intakePower;
    }

    public double getFeedPower() {
        return feedPower;
    }

    public void stop() {
        setPowers(0, 0);
    }

    public void setLiftPosition(double position) {
        liftPosition = Range.clip(position, IntakeConstants.LIFT_MIN, IntakeConstants.LIFT_MAX);
        liftServo.setPosition(liftPosition);
        mirroredLiftServo.setPosition(1.0 - liftPosition);
    }
}
