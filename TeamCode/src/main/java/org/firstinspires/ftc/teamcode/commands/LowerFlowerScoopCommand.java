package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.flowerscoop.FlowerScoop;

public final class LowerFlowerScoopCommand extends CommandBase {
    private final FlowerScoop flowerScoop;

    public LowerFlowerScoopCommand(FlowerScoop flowerScoop) {
        this.flowerScoop = flowerScoop;
        addRequirements(flowerScoop);
    }

    @Override
    public void initialize() {
        flowerScoop.lower();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
