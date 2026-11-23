package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

public final class ShooterCommand extends CommandBase {
    private final Shooter shooter;
    private final DoubleSupplier runtime;

    public ShooterCommand(Shooter shooter, DoubleSupplier runtime) {
        this.shooter = shooter;
        this.runtime = runtime;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.startControl(runtime.getAsDouble());
    }

    @Override
    public void execute() {
        shooter.updateControl(runtime.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
