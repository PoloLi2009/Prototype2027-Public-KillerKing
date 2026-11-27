package org.firstinspires.ftc.teamcode.subsystems.flowerscoop;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Flower scoop driven by the mirrored s3 / s03 servo pair. Starts raised. */
public final class FlowerScoop extends SubsystemBase {
    private final Servo servo;
    private final Servo mirroredServo;
    private boolean raised;

    public FlowerScoop(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, FlowerScoopConstants.SERVO_NAME);
        mirroredServo = hardwareMap.get(Servo.class, FlowerScoopConstants.MIRRORED_SERVO_NAME);
        raise();
    }

    public void raise() {
        setRaised(true);
    }

    public void lower() {
        setRaised(false);
    }

    public boolean isRaised() {
        return raised;
    }

    /** Returns the s3 position; s03 is at 1.0 minus this value. */
    public double getPosition() {
        return raised ? FlowerScoopConstants.RAISED_POSITION : FlowerScoopConstants.LOWERED_POSITION;
    }

    private void setRaised(boolean raised) {
        this.raised = raised;
        double position = getPosition();
        servo.setPosition(position);
        mirroredServo.setPosition(1.0 - position);
    }
}
