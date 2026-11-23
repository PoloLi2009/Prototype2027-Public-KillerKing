package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.gate.Gate;
import org.firstinspires.ftc.teamcode.subsystems.gate.GateConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants;

import java.util.function.DoubleSupplier;

public final class FeedCommand extends CommandBase {
    private final Intake intake;
    private final Gate gate;
    private final Gamepad gamepad;
    private final DoubleSupplier runtime;
    private double firePower = IntakeConstants.INITIAL_FEED_POWER;
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
        boolean firing = gamepad.right_trigger > IntakeConstants.TRIGGER_DEADZONE;
        boolean intaking = gamepad.left_trigger > IntakeConstants.TRIGGER_DEADZONE;
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
            if (now - fireStartTime >= GateConstants.PRE_OPEN_SECONDS) {
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
        firePower = Range.clip(firePower - IntakeConstants.FEED_POWER_STEP,
                IntakeConstants.MIN_FEED_POWER, IntakeConstants.MAX_FEED_POWER);
    }

    public void increaseFirePower() {
        firePower = Range.clip(firePower + IntakeConstants.FEED_POWER_STEP,
                IntakeConstants.MIN_FEED_POWER, IntakeConstants.MAX_FEED_POWER);
    }

    public double getFirePower() {
        return firePower;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
