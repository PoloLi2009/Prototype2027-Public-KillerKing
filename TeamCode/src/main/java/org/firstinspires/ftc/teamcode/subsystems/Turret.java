package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public final class Turret extends SubsystemBase {
    public static final double MAX_OFFSET_DEGREES = 135.0;
    private static final double CENTER_POSITION = 0.5;
    private static final double SERVO_TRAVEL_DEGREES = 350.0;

    private final Servo firstServo;
    private final Servo secondServo;
    private double offsetDegrees;

    public Turret(HardwareMap hardwareMap) {
        firstServo = hardwareMap.get(Servo.class, "s2");
        secondServo = hardwareMap.get(Servo.class, "s01");
        setOffsetDegrees(0);
    }

    public void setOffsetDegrees(double degrees) {
        offsetDegrees = Range.clip(degrees, -MAX_OFFSET_DEGREES, MAX_OFFSET_DEGREES);
        double position = getPosition();
        firstServo.setPosition(position);
        secondServo.setPosition(position);
    }

    public double getOffsetDegrees() {
        return offsetDegrees;
    }

    public double getPosition() {
        // Positive rotation from rear center decreases both servo positions.
        return CENTER_POSITION - offsetDegrees / SERVO_TRAVEL_DEGREES;
    }
}
