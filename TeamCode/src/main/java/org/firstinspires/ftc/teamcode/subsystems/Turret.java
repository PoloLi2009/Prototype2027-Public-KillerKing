package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public final class Turret extends SubsystemBase {
    private static final double STEP = 0.1;
    private static final double MIN_POSITION = 0.2;
    private static final double MAX_POSITION = 0.8;

    private final Servo firstServo;
    private final Servo secondServo;
    private double position = 0.5;

    public Turret(HardwareMap hardwareMap) {
        firstServo = hardwareMap.get(Servo.class, "s2");
        secondServo = hardwareMap.get(Servo.class, "s01");
        setPosition(position);
    }

    public void increasePosition() {
        setPosition(position + STEP);
    }

    public void decreasePosition() {
        setPosition(position - STEP);
    }

    public double getPosition() {
        return position;
    }

    private void setPosition(double position) {
        this.position = Range.clip(position, MIN_POSITION, MAX_POSITION);
        firstServo.setPosition(this.position);
        secondServo.setPosition(this.position);
    }
}
