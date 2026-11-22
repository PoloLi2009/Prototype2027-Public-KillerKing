package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

public final class DriveCommand extends CommandBase {
    private static final double DEADZONE = 0.05;

    private final Drive drive;
    private final Gamepad gamepad;

    public DriveCommand(Drive drive, Gamepad gamepad) {
        this.drive = drive;
        this.gamepad = gamepad;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.driveFieldCentric(deadzone(-gamepad.left_stick_y),
                deadzone(gamepad.left_stick_x), deadzone(gamepad.right_stick_x));
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    private static double deadzone(double input) {
        return Math.abs(input) < DEADZONE ? 0 : input;
    }
}
