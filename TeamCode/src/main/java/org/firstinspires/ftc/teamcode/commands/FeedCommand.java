package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public final class FeedCommand extends CommandBase {
    private static final double TRIGGER_DEADZONE = 0.01;
    private static final double FIRE_POWER_STEP = 0.1;

    private final Intake intake;
    private final Gamepad gamepad;
    private double firePower = 1.0;

    public FeedCommand(Intake intake, Gamepad gamepad) {
        this.intake = intake;
        this.gamepad = gamepad;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        if (gamepad.right_bumper) {
            intake.setPower(firePower);
        } else if (gamepad.right_trigger > TRIGGER_DEADZONE) {
            intake.setPower(gamepad.right_trigger);
        } else if (gamepad.left_trigger > TRIGGER_DEADZONE) {
            intake.setPower(-gamepad.left_trigger);
        } else {
            intake.stop();
        }
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
