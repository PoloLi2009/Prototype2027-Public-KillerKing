package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public final class Turret extends SubsystemBase {
    private final Servo firstServo;
    private final Servo secondServo;
    private double offsetDegrees;
    private double estimatedOffsetDegrees;
    private double estimateTimeSeconds = Double.NaN;

    public Turret(HardwareMap hardwareMap) {
        firstServo = hardwareMap.get(Servo.class, TurretConstants.FIRST_SERVO_NAME);
        secondServo = hardwareMap.get(Servo.class, TurretConstants.SECOND_SERVO_NAME);
        setOffsetDegrees(0);
    }

    public void setOffsetDegrees(double degrees) {
        getEstimatedOffsetDegrees(System.nanoTime() / 1e9);
        offsetDegrees = Range.clip(degrees,
                -TurretConstants.MAX_OFFSET_DEGREES, TurretConstants.MAX_OFFSET_DEGREES);
        double position = getPosition();
        firstServo.setPosition(position);
        secondServo.setPosition(position);
    }

    public double getOffsetDegrees() {
        return offsetDegrees;
    }

    public void setPosition(double position) {
        double offset = (Range.clip(position, 0, 1) - TurretConstants.CENTER_POSITION)
                * TurretConstants.SERVO_TRAVEL_DEGREES / TurretConstants.SERVO_DIRECTION;
        setOffsetDegrees(offset);
    }

    public double getEstimatedOffsetDegrees(double nowSeconds) {
        if (Double.isFinite(estimateTimeSeconds)) {
            if (nowSeconds <= estimateTimeSeconds) {
                return estimatedOffsetDegrees;
            }
            double travel = (nowSeconds - estimateTimeSeconds)
                    * TurretConstants.ESTIMATED_SPEED_DEGREES_PER_SECOND;
            estimatedOffsetDegrees += Range.clip(offsetDegrees - estimatedOffsetDegrees,
                    -travel, travel);
        }
        estimateTimeSeconds = nowSeconds;
        return estimatedOffsetDegrees;
    }

    public double getPosition() {
        return TurretConstants.CENTER_POSITION
                + TurretConstants.SERVO_DIRECTION * offsetDegrees / TurretConstants.SERVO_TRAVEL_DEGREES;
    }
}
