package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

public final class ShooterCommand extends CommandBase {
    private final Shooter shooter;
    private final DoubleSupplier runtime;
    private final DoubleSupplier hiveDistanceCm;

    public ShooterCommand(Shooter shooter, DoubleSupplier runtime, DoubleSupplier hiveDistanceCm) {
        this.shooter = shooter;
        this.runtime = runtime;
        this.hiveDistanceCm = hiveDistanceCm;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.startControl(runtime.getAsDouble());
    }

    @Override
    public void execute() {
        shooter.updateControl(runtime.getAsDouble(), hiveDistanceCm.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
}
