package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public final class Turret extends SubsystemBase {
    private final Servo firstServo;
    private final Servo secondServo;
    private double offsetDegrees;

    public Turret(HardwareMap hardwareMap) {
        firstServo = hardwareMap.get(Servo.class, TurretConstants.FIRST_SERVO_NAME);
        secondServo = hardwareMap.get(Servo.class, TurretConstants.SECOND_SERVO_NAME);
        setOffsetDegrees(0);
    }

    public void setOffsetDegrees(double degrees) {
        offsetDegrees = Range.clip(degrees,
                -TurretConstants.MAX_OFFSET_DEGREES, TurretConstants.MAX_OFFSET_DEGREES);
        double position = getPosition();
        firstServo.setPosition(position);
        secondServo.setPosition(position);
    }

    public double getOffsetDegrees() {
        return offsetDegrees;
    }

    public double getPosition() {
        // Positive rotation from rear center decreases both servo positions.
        return TurretConstants.CENTER_POSITION - offsetDegrees / TurretConstants.SERVO_TRAVEL_DEGREES;
    }
}
