package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.function.DoubleSupplier;

public final class FeedCommand extends CommandBase {
    private static final double TRIGGER_DEADZONE = 0.01;
    private static final double FIRE_POWER_STEP = 0.1;
    private static final double FIRE_PRE_OPEN_SEC = 0.1;

    private final Intake intake;
    private final Gate gate;
    private final Gamepad gamepad;
    private final DoubleSupplier runtime;
    private double firePower = 1.0;
    private double fireStartTime;
    private boolean wasFiring;
    private boolean wasIntaking;

    public FeedCommand(Intake intake, Gate gate, Gamepad gamepad, DoubleSupplier runtime) {
        this.intake = intake;
        this.gate = gate;
        this.gamepad = gamepad;
        this.runtime = runtime;
        addRequirements(intake, gate);
    }

    @Override
    public void initialize() {
        wasFiring = false;
        wasIntaking = false;
    }

    @Override
    public void execute() {
        double now = runtime.getAsDouble();
        boolean firing = gamepad.right_trigger > TRIGGER_DEADZONE;
        boolean intaking = gamepad.left_trigger > TRIGGER_DEADZONE;
        boolean firePressed = firing && !wasFiring;
        if (firePressed) {
            fireStartTime = now;
        }
        // Intake wins when both triggers request a gate change in the same loop.
        if (intaking && !wasIntaking) {
            gate.setOpen(false);
        } else if (firePressed) {
            gate.setOpen(true);
        }

        double power = 0;
        if (firing) {
            if (now - fireStartTime >= FIRE_PRE_OPEN_SEC) {
                power = firePower;
            }
        } else if (intaking) {
            power = firePower;
        } else if (gamepad.left_bumper) {
            power = -firePower;
        }
        intake.setPower(power);
        wasFiring = firing;
        wasIntaking = intaking;
    }

    public void decreaseFirePower() {
        firePower = Range.clip(firePower - FIRE_POWER_STEP, 0, 1);
    }

    public void increaseFirePower() {
        firePower = Range.clip(firePower + FIRE_POWER_STEP, 0, 1);
    }

    public double getFirePower() {
        return firePower;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
