package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FeedCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.commands.TurretAimCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.function.BooleanSupplier;

@TeleOp(name = "Ba", group = "Test")
public final class KillerKingTeleOp extends CommandOpMode {
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Gate gate;
    private Pinpoint pinpoint;
    private FeedCommand feedCommand;
    private TurretAimCommand turretAimCommand;

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        pinpoint = new Pinpoint(hardwareMap);
        Turret turret = new Turret(hardwareMap);

        drive.setDefaultCommand(new DriveCommand(drive, gamepad1));
        shooter.setDefaultCommand(new ShooterCommand(shooter, this::getRuntime));
        feedCommand = new FeedCommand(intake, gate, gamepad1, this::getRuntime);
        intake.setDefaultCommand(feedCommand);
        turretAimCommand = new TurretAimCommand(pinpoint, turret);
        turret.setDefaultCommand(turretAimCommand);

        onPress(() -> gamepad1.dpad_left, shooter::decreaseRpm);
        onPress(() -> gamepad1.dpad_right, shooter::increaseRpm);
        onPress(() -> gamepad1.dpad_up, intake::raiseLift);
        onPress(() -> gamepad1.dpad_down, intake::lowerLift);
        onPress(() -> gamepad1.a, feedCommand::decreaseFirePower);
        onPress(() -> gamepad1.b, feedCommand::increaseFirePower);
        onPress(() -> gamepad1.y, turretAimCommand::toggleAim);
        onPress(() -> gamepad1.start, drive::resetYaw);

        telemetry.addLine("Killer King 初始化完成");
        telemetry.addLine("←/→ 调 RPM  LT 吸球  RT 射球  LB 反转  A/B 调功率");
        telemetry.addLine("↑/↓ 抬升  Y 自瞄  START 重置驾驶朝向");
        pinpoint.addTelemetry(telemetry);
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
            schedule(drive.getDefaultCommand(), shooter.getDefaultCommand(), feedCommand,
                    turretAimCommand);
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
        pinpoint.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        telemetry.addData("Intake 抬升", "%.2f", intake.getLiftPosition());
        turretAimCommand.addTelemetry(telemetry);
        telemetry.addData("球挡", "%s (%.2f)", gate.isOpen() ? "开" : "关", gate.getPosition());
        telemetry.addData("吸球/送料/反转功率 (A/B)", "%.1f", feedCommand.getFirePower());
        telemetry.addData("Intake/通道 功率", "%.2f", intake.getPower());
        telemetry.update();
    }

    private void onPress(BooleanSupplier pressed, Runnable action) {
        new Trigger(() -> isStarted() && pressed.getAsBoolean()).whenActive(action);
    }
}
