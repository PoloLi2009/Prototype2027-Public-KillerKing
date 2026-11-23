package org.firstinspires.ftc.teamcode.subsystems.gate;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public final class Gate extends SubsystemBase {
    private final Servo servo;
    private boolean open;

    public Gate(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, GateConstants.SERVO_NAME);
        servo.setPosition(GateConstants.CLOSED_POSITION);
    }

    public void setOpen(boolean open) {
        this.open = open;
        servo.setPosition(getPosition());
    }

    public boolean isOpen() {
        return open;
    }

    public double getPosition() {
        return open ? GateConstants.OPEN_POSITION : GateConstants.CLOSED_POSITION;
    }
}
