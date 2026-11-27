package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.gate.Gate;
import org.firstinspires.ftc.teamcode.subsystems.gate.GateConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public final class FeedCommand extends CommandBase {
    private final Intake intake;
    private final Gate gate;
    private final Gamepad gamepad;
    private final DoubleSupplier runtime;
    private final BooleanSupplier flywheelReady;
    private double fireStartTime;
    private boolean feedStarted;                      // latched: once a volley feeds, it never pauses
    private double feedWaitStartTime = Double.NaN;    // NaN until the volley starts waiting for the flywheel
    private boolean wasFiring;
    private boolean wasIntaking;

    /**
     * @param flywheelReady Shooter.isReadyToFeed(): when RT is pressed, feeding starts once it is true
     *                      (or after ShooterConstants.FEED_MAX_WAIT_SECONDS) and then runs without a
     *                      stop until RT is released.
     */
    public FeedCommand(Intake intake, Gate gate, Gamepad gamepad, DoubleSupplier runtime,
                       BooleanSupplier flywheelReady) {
        this.intake = intake;
        this.gate = gate;
        this.gamepad = gamepad;
        this.runtime = runtime;
        this.flywheelReady = flywheelReady;
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
        if (!firing) {
            feedStarted = false;
            feedWaitStartTime = Double.NaN;
        }
        if (firePressed) {
            gate.setOpen(true);
        } else if (!firing && intaking && (!wasIntaking || wasFiring)) {
            gate.setOpen(false);
        }

        // 1 runs both motors inward, -1 reverses them, 0 stops them. Firing uses its own powers.
        int direction = 0;
        double intakePower = IntakeConstants.INTAKE_MOTOR_POWER;
        double feedPower = IntakeConstants.FEED_MOTOR_POWER;
        if (firing) {
            intakePower = IntakeConstants.FIRE_INTAKE_MOTOR_POWER;
            feedPower = IntakeConstants.FIRE_FEED_MOTOR_POWER;
            if (now - fireStartTime >= GateConstants.PRE_OPEN_SECONDS) {
                direction = feedAllowed(now) ? 1 : 0;
            }
        } else if (intaking) {
            direction = 1;
        } else if (gamepad.left_bumper) {
            direction = -1;
        }
        intake.setPowers(direction * intakePower, direction * feedPower);
        wasFiring = firing;
        wasIntaking = intaking;
    }

    /**
     * The first ball of each RT press waits for the flywheel, at most FEED_MAX_WAIT_SECONDS (a flywheel
     * that cannot reach its target must not stop the driver from shooting). Once feeding has started
     * it does not pause again, whatever the flywheel does, until RT is released.
     */
    private boolean feedAllowed(double now) {
        if (feedStarted) return true;
        if (Double.isNaN(feedWaitStartTime)) {
            feedWaitStartTime = now;
        }
        feedStarted = flywheelReady.getAsBoolean()
                || now - feedWaitStartTime >= ShooterConstants.FEED_MAX_WAIT_SECONDS;
        return feedStarted;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
