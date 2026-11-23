package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public final class Gate extends SubsystemBase {
    private static final double CLOSED_POSITION = 0.55;
    private static final double OPEN_POSITION = 0.8;

    private final Servo servo;
    private boolean open;

    public Gate(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "s1");
        servo.setPosition(CLOSED_POSITION);
    }

    public void setOpen(boolean open) {
        this.open = open;
        servo.setPosition(getPosition());
    }

    public boolean isOpen() {
        return open;
    }

    public double getPosition() {
        return open ? OPEN_POSITION : CLOSED_POSITION;
    }
}
