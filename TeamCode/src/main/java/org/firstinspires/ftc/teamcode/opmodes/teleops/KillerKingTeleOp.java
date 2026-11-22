package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FeedCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.function.BooleanSupplier;

@TeleOp(name = "Ba", group = "Test")
public final class KillerKingTeleOp extends CommandOpMode {
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Turret turret;
    private Gate gate;
    private FeedCommand feedCommand;

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        turret = new Turret(hardwareMap);
        gate = new Gate(hardwareMap);

        drive.setDefaultCommand(new DriveCommand(drive, gamepad1));
        shooter.setDefaultCommand(new ShooterCommand(shooter, this::getRuntime));
        feedCommand = new FeedCommand(intake, gamepad1);
        intake.setDefaultCommand(feedCommand);

        onPress(() -> gamepad1.dpad_left, shooter::decreaseRpm);
        onPress(() -> gamepad1.dpad_right, shooter::increaseRpm);
        onPress(() -> gamepad1.dpad_up, intake::raiseLift);
        onPress(() -> gamepad1.dpad_down, intake::lowerLift);
        onPress(() -> gamepad1.left_bumper, gate::toggle);
        onPress(() -> gamepad1.a, feedCommand::decreaseFirePower);
        onPress(() -> gamepad1.b, feedCommand::increaseFirePower);
        onPress(() -> gamepad1.x, shooter::clearRpm);
        onPress(() -> gamepad1.y, turret::increasePosition);
        onPress(() -> gamepad1.back, turret::decreasePosition);
        onPress(() -> gamepad1.start, drive::resetYaw);

        telemetry.addLine("Killer King 初始化完成");
        telemetry.addLine("←/→ 调 RPM  LB 扳机  RB 按住发射  A/B 调发射功率  X 清零");
        telemetry.addLine("↑/↓ 抬升  Y/BACK 云台  RT/LT 吸吐  START 重置朝向");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        reset();
        try {
            initialize();
            waitForStart();
            if (isStopRequested()) {
                return;
            }

            drive.resetYaw();
            schedule(drive.getDefaultCommand(), shooter.getDefaultCommand(), feedCommand);
            while (opModeIsActive()) {
                run();
            }
        } finally {
            reset();
            if (drive != null) {
                drive.stop();
            }
            if (shooter != null) {
                shooter.stop();
            }
            if (intake != null) {
                intake.stop();
            }
        }
    }

    @Override
    public void run() {
        super.run();

        drive.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        telemetry.addData("Intake 抬升", "%.2f", intake.getLiftPosition());
        telemetry.addData("云台", "%.2f", turret.getPosition());
        telemetry.addData("扳机", "%s (%.2f)", gate.isOpen() ? "开" : "关", gate.getPosition());
        telemetry.addData("发射功率 (A/B)", "%.1f", feedCommand.getFirePower());
        telemetry.addData("RT / LT", "%.2f / %.2f", gamepad1.right_trigger, gamepad1.left_trigger);
        telemetry.addData("RB 发射中", gamepad1.right_bumper ? "是" : "否");
        telemetry.addData("Intake/通道 功率", "%.2f", intake.getPower());
        telemetry.update();
    }

    private void onPress(BooleanSupplier pressed, Runnable action) {
        new Trigger(() -> isStarted() && pressed.getAsBoolean()).whenActive(action);
    }
}
