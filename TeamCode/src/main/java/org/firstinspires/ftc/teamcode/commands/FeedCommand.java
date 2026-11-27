package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

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
        if (firePressed) {
            gate.setOpen(true);
        } else if (!firing && intaking && (!wasIntaking || wasFiring)) {
            gate.setOpen(false);
        }

        // 1 runs both motors inward, -1 reverses them, 0 stops them.
        int direction = 0;
        if (firing) {
            if (now - fireStartTime >= GateConstants.PRE_OPEN_SECONDS) {
                direction = 1;
            }
        } else if (intaking) {
            direction = 1;
        } else if (gamepad.left_bumper) {
            direction = -1;
        }
        intake.setPowers(direction * IntakeConstants.INTAKE_MOTOR_POWER,
                direction * IntakeConstants.FEED_MOTOR_POWER);
        wasFiring = firing;
        wasIntaking = intaking;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
