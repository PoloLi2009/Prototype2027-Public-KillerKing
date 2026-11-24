package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FeedCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.commands.TurretAimCommand;
import org.firstinspires.ftc.teamcode.commands.VisionReadCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.gate.Gate;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.localization.VisionLocalization;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;

import java.util.function.BooleanSupplier;

@TeleOp(name = "Ba", group = "Test")
public class KillerKingTeleOp extends CommandOpMode {
    private final Alliance alliance;
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Gate gate;
    private Pinpoint pinpoint;
    private LimelightVision vision;
    private FeedCommand feedCommand;
    private VisionReadCommand visionReadCommand;
    private VisionLocalization localization;
    private TurretAimCommand turretAimCommand;

    public KillerKingTeleOp() {
        this(Alliance.BLUE);
    }

    protected KillerKingTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        pinpoint = new Pinpoint(hardwareMap);
        Turret turret = new Turret(hardwareMap);
        vision = new LimelightVision(hardwareMap, alliance);
        localization = new VisionLocalization();
        visionReadCommand = new VisionReadCommand(vision, pinpoint, turret, localization);
        vision.setDefaultCommand(visionReadCommand);

        drive.setDefaultCommand(new DriveCommand(drive, gamepad1));
        shooter.setDefaultCommand(new ShooterCommand(shooter, this::getRuntime));
        feedCommand = new FeedCommand(intake, gate, gamepad1, this::getRuntime);
        intake.setDefaultCommand(feedCommand);
        turretAimCommand = new TurretAimCommand(visionReadCommand::getAimPose, vision::isBlueAlliance, turret);
        turret.setDefaultCommand(turretAimCommand);

        onPress(() -> gamepad1.dpad_left, shooter::decreaseRpm);
        onPress(() -> gamepad1.dpad_right, shooter::increaseRpm);
        onPress(() -> gamepad1.dpad_up, intake::raiseLift);
        onPress(() -> gamepad1.dpad_down, intake::lowerLift);
        onPress(() -> gamepad1.a, feedCommand::decreaseFirePower);
        onPress(() -> gamepad1.b, feedCommand::increaseFirePower);
        onPress(() -> gamepad1.y, turretAimCommand::toggleAim);
        onPress(() -> gamepad1.right_stick_button, visionReadCommand::toggleVisionForAim);
        onPress(() -> gamepad1.start, drive::resetYaw);

        telemetry.addLine("Killer King 初始化完成");
        telemetry.addLine("←/→ 调 RPM  LT 吸球  RT 射球  LB 反转  A/B 调功率");
        telemetry.addLine("↑/↓ 抬升  Y 自瞄  START 重置驾驶朝向");
        telemetry.addLine("右摇杆按下：切换自瞄位姿 Pinpoint / EKF");
        pinpoint.addTelemetry(telemetry);
        visionReadCommand.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        reset();
        try {
            initialize();
            while (opModeInInit()) {
                vision.preparePipeline(this::opModeInInit);
                telemetry.addData("Limelight INIT", "P%d / %s",
                        vision.getExpectedPipeline(), vision.getStatus());
                telemetry.update();
                sleep(VisionConstants.pipelineInitPollMs);
            }
            vision.finishPipelineSetup();
            if (isStopRequested()) {
                return;
            }

            drive.resetYaw();
            schedule(drive.getDefaultCommand(), shooter.getDefaultCommand(), feedCommand,
                    visionReadCommand, turretAimCommand);
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
        }
    }

    @Override
    public void run() {
        super.run();

        drive.addTelemetry(telemetry);
        pinpoint.addTelemetry(telemetry);
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

    private void onPress(BooleanSupplier pressed, Runnable action) {
        new Trigger(() -> isStarted() && pressed.getAsBoolean()).whenActive(action);
    }
}
