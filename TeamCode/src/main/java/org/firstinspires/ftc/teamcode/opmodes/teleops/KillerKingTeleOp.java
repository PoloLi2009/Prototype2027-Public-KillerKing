package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FeedCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.commands.TurretAimCommand;
import org.firstinspires.ftc.teamcode.commands.VisionReadCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.gate.Gate;
import org.firstinspires.ftc.teamcode.subsystems.hivelights.HiveLights;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.localization.VisionLocalization;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;

import java.util.function.BooleanSupplier;

public abstract class KillerKingTeleOp extends CommandOpMode {
    private final Alliance alliance;
    private final boolean tuneShooter;
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Gate gate;
    private HiveLights hiveLights;
    private LimelightVision vision;
    private FeedCommand feedCommand;
    private VisionReadCommand visionReadCommand;
    private VisionLocalization localization;
    private TurretAimCommand turretAimCommand;

    protected KillerKingTeleOp(Alliance alliance, boolean tuneShooter) {
        this.alliance = alliance;
        this.tuneShooter = tuneShooter;
    }

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        if (!tuneShooter) {
            shooter.toggleDistanceRpm();
        }
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        hiveLights = new HiveLights(hardwareMap);
        Pinpoint pinpoint = new Pinpoint(hardwareMap);
        hiveLights.setDefaultCommand(new RunCommand(() -> hiveLights.showDistance(
                pinpoint.getRawOdometry(), alliance == Alliance.BLUE), hiveLights));
        Turret turret = new Turret(hardwareMap);
        vision = new LimelightVision(hardwareMap, alliance);
        localization = new VisionLocalization();
        visionReadCommand = new VisionReadCommand(vision, pinpoint, turret, localization);
        vision.setDefaultCommand(visionReadCommand);

        drive.setDefaultCommand(new DriveCommand(drive, gamepad1));
        feedCommand = new FeedCommand(intake, gate, gamepad1, this::getRuntime);
        intake.setDefaultCommand(feedCommand);
        turretAimCommand = new TurretAimCommand(visionReadCommand::getAimPose, vision::isBlueAlliance, turret);
        turret.setDefaultCommand(turretAimCommand);
        shooter.setDefaultCommand(new ShooterCommand(shooter, this::getRuntime, turretAimCommand::getTargetDistanceCm));

        onPress(() -> gamepad1.dpad_left, () -> shooter.adjustRpm(-1));
        onPress(() -> gamepad1.dpad_right, () -> shooter.adjustRpm(1));
        onPress(() -> gamepad1.dpad_up, intake::raiseLift);
        onPress(() -> gamepad1.dpad_down, intake::lowerLift);
        onPress(() -> gamepad1.a, feedCommand::decreaseFirePower);
        onPress(() -> gamepad1.b, feedCommand::increaseFirePower);
        onPress(() -> gamepad1.y, turretAimCommand::toggleAim);
        onPress(() -> gamepad1.start, drive::resetYaw);

        telemetry.addLine("Killer King 初始化完成");
        telemetry.addLine("←/→ 调 RPM  LT 吸球  RT 射球  LB 反转  A/B 调功率");
        telemetry.addLine("↑/↓ 抬升  三角/Y 切换自瞄（默认开）  START 重置驾驶朝向");
        if (tuneShooter) {
            bindShooterTuning();
            addShooterTuningTelemetry();
        } else {
            visionReadCommand.addTelemetry(telemetry);
        }
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        reset();
        try {
            initialize();
            while (opModeInInit()) {
                vision.preparePipeline(this::opModeInInit);
                if (tuneShooter) {
                    addShooterTuningTelemetry();
                } else {
                    telemetry.addData("Limelight INIT", "P%d / %s",
                            vision.getExpectedPipeline(), vision.getStatus());
                }
                telemetry.update();
                sleep(VisionConstants.pipelineInitPollMs);
            }
            vision.finishPipelineSetup();
            if (isStopRequested()) {
                return;
            }

            drive.resetYaw();
            schedule(drive.getDefaultCommand(), visionReadCommand,
                    turretAimCommand, shooter.getDefaultCommand(), feedCommand,
                    hiveLights.getDefaultCommand());
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
            if (vision != null) {
                vision.stop();
            }
            if (hiveLights != null) {
                hiveLights.turnOff();
            }
        }
    }

    @Override
    public void run() {
        super.run();

        if (tuneShooter) {
            addShooterTuningTelemetry();
            telemetry.update();
            return;
        }

        drive.addTelemetry(telemetry);
        visionReadCommand.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        double hiveTiltDegrees = localization.getHiveTiltDegrees();
        if (Double.isFinite(hiveTiltDegrees)) {
            telemetry.addData("HIVE motion / tilt", "%s / %+.1f°",
                    localization.getHiveMotionState(), hiveTiltDegrees);
        } else {
            telemetry.addData("HIVE motion / tilt", "UNKNOWN / No fresh tilt");
        }
        telemetry.addData("Intake 抬升", "%.2f", intake.getLiftPosition());
        turretAimCommand.addTelemetry(telemetry);
        telemetry.addData("球挡", "%s (%.2f)", gate.isOpen() ? "开" : "关", gate.getPosition());
        telemetry.addData("吸球/送料/反转功率 (A/B)", "%.1f", feedCommand.getFirePower());
        telemetry.addData("Intake/通道 功率", "%.2f", intake.getPower());
        telemetry.update();
    }

    private void bindShooterTuning() {
        onPress(() -> gamepad2.dpad_left, () -> shooter.selectGain(-1));
        onPress(() -> gamepad2.dpad_right, () -> shooter.selectGain(1));
        onPress(() -> gamepad2.dpad_up, () -> shooter.adjustSelectedGain(1));
        onPress(() -> gamepad2.dpad_down, () -> shooter.adjustSelectedGain(-1));
        onPress(() -> gamepad2.a, shooter::toggleManualPower);
        onPress(() -> gamepad2.x, shooter::toggleDistanceRpm);
        onPress(() -> gamepad2.left_bumper, () -> shooter.adjustManualPower(-1));
        onPress(() -> gamepad2.right_bumper, () -> shooter.adjustManualPower(1));
    }

    private void addShooterTuningTelemetry() {
        shooter.addTelemetry(telemetry);
        shooter.addTuningTelemetry(telemetry);
        double hiveDistanceCm = turretAimCommand.getTargetDistanceCm();
        if (Double.isFinite(hiveDistanceCm)) {
            telemetry.addData("到目标 Hive", "%.1f cm", hiveDistanceCm);
        } else {
            telemetry.addData("到目标 Hive", "等待位姿");
        }
        telemetry.addData("送料设定 / 实际", "%.2f / %.2f", feedCommand.getFirePower(), intake.getPower());
        telemetry.addData("球挡", gate.isOpen() ? "开" : "关");
        telemetry.addLine("手柄 2  ←/→ 选增益  ↑/↓ 调增益  A 切模式  LB/RB 调手动功率");
        telemetry.addLine("手柄 2 X 切换固定 RPM / 距离插值");
    }

    private void onPress(BooleanSupplier pressed, Runnable action) {
        new Trigger(() -> isStarted() && pressed.getAsBoolean()).whenActive(action);
    }
}
