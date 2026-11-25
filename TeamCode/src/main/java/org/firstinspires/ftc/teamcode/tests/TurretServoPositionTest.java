package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@TeleOp(name = "Turret Servo Position Test", group = "Quest")
public final class TurretServoPositionTest extends CommandOpMode {
    private Turret turret;

    @Override
    public void initialize() {
        turret = new Turret(hardwareMap);
        new Trigger(() -> gamepad1.dpad_up).whenActive(new InstantCommand(() -> stepPosition(1), turret));
        new Trigger(() -> gamepad1.dpad_down).whenActive(new InstantCommand(() -> stepPosition(-1), turret));
        telemetry.addLine("手柄 1 D-pad ↑ +0.01，↓ -0.01");
        showPosition();
    }

    @Override
    public void run() {
        super.run();
        showPosition();
    }

    private void stepPosition(int hundredths) {
        long currentHundredths = Math.round(turret.getPosition() * 100);
        turret.setPosition((currentHundredths + hundredths) / 100.0);
    }

    private void showPosition() {
        telemetry.addData("云台舵机指令位置", "%.2f", turret.getPosition());
        telemetry.update();
    }
}
