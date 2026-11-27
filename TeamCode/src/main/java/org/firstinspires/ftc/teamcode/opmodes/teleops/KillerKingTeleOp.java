package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.FeedCommand;
import org.firstinspires.ftc.teamcode.commands.LowerFlowerScoopCommand;
import org.firstinspires.ftc.teamcode.commands.RaiseFlowerScoopCommand;
import org.firstinspires.ftc.teamcode.commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.commands.TurretAimCommand;
import org.firstinspires.ftc.teamcode.commands.VisionReadCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.flowerscoop.FlowerScoop;
import org.firstinspires.ftc.teamcode.subsystems.gate.Gate;
import org.firstinspires.ftc.teamcode.subsystems.hivelights.HiveLights;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.localization.VisionLocalization;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.AutoEndPose;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.PinpointConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;

import java.util.function.BooleanSupplier;

public abstract class KillerKingTeleOp extends CommandOpMode {
    private final Alliance alliance;
    private final boolean tuneShooter;
    // A TeleOp that follows a particular autonomous: its preset start pose, and the autonomous whose
    // measured end pose (AutoEndPose) replaces it when fresh. Null for the other TeleOps.
    private final Pose2D presetStartPose;
    private final String afterAutonomous;
    private String startPoseSource = "preset";
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Gate gate;
    private FlowerScoop flowerScoop;
    private HiveLights hiveLights;
    private LimelightVision vision;
    private FeedCommand feedCommand;
    private VisionReadCommand visionReadCommand;
    private VisionLocalization localization;
    private TurretAimCommand turretAimCommand;
    private boolean deadZoneRumbling;
    private Pose2D startPose;
    // Loop time telemetry, for the Shooter Auto Tuner loop period: a running average, and the longest
    // loop of the last LOOP_PEAK_WINDOW_NS.
    private static final long LOOP_PEAK_WINDOW_NS = 2_000_000_000L;
    private long lastLoopNs;
    private long loopWindowStartNs;
    private double loopMsAverage = Double.NaN;
    private double loopMsWindowMax;
    private double loopMsShownMax = Double.NaN;

    protected KillerKingTeleOp(Alliance alliance, boolean tuneShooter) {
        this(alliance, tuneShooter, null, null);
    }

    /**
     * A TeleOp that starts where {@code afterAutonomous} ended: its measured end pose if it handed one
     * over within AutoEndPose.MAX_AGE_SECONDS, else {@code presetStartPose}.
     */
    protected KillerKingTeleOp(Alliance alliance, Pose2D presetStartPose, String afterAutonomous) {
        this(alliance, false, presetStartPose, afterAutonomous);
    }

    private KillerKingTeleOp(Alliance alliance, boolean tuneShooter, Pose2D presetStartPose,
                             String afterAutonomous) {
        this.alliance = alliance;
        this.tuneShooter = tuneShooter;
        this.presetStartPose = presetStartPose;
        this.afterAutonomous = afterAutonomous;
    }

    protected String previousAutonomous() {
        return afterAutonomous;
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
        flowerScoop = new FlowerScoop(hardwareMap);
        hiveLights = new HiveLights(hardwareMap);
        // Red / Blue start where their AUTO parks; the Limelight tuning TeleOps keep the field center.
        // A TeleOp after a particular AUTO prefers that AUTO's measured end pose.
        String source = previousAutonomous();
        if (source != null) {
            Pose2D handed = AutoEndPose.recent(source);
            startPose = handed != null ? handed : presetStartPose;
            startPoseSource = handed != null
                    ? String.format("%s 实测终点（%.0f s 前）", source, AutoEndPose.ageSeconds())
                    : String.format("%s 预设终点（没有 2 分钟内的实测终点）", source);
        } else {
            startPose = tuneShooter ? PinpointConstants.START_POSE
                    : alliance == Alliance.RED ? PinpointConstants.RED_TELEOP_START_POSE
                    : PinpointConstants.BLUE_TELEOP_START_POSE;
        }
        Pinpoint pinpoint = new Pinpoint(hardwareMap, startPose);
        hiveLights.setDefaultCommand(new RunCommand(() -> hiveLights.showDistance(
                pinpoint.getRawOdometry(), alliance == Alliance.BLUE,
                turretAimCommand != null && turretAimCommand.isInDeadZone()), hiveLights));
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
        onPress(() -> gamepad1.b, visionReadCommand::forceVisionAlignment);
        // The co-driver can force the same Limelight alignment (gamepad 2 circle / B is otherwise unused).
        onPress(() -> gamepad2.b, visionReadCommand::forceVisionAlignment);
        onPress(() -> gamepad1.y, turretAimCommand::toggleAim);
        onPress(() -> gamepad1.x, drive::resetYaw);
        scheduleOnPress(() -> gamepad1.right_stick_button, new ConditionalCommand(
                new LowerFlowerScoopCommand(flowerScoop),
                new RaiseFlowerScoopCommand(flowerScoop),
                flowerScoop::isRaised));

        telemetry.addLine("Killer King 初始化完成");
        telemetry.addLine(String.format("Pinpoint 初始位姿 (%.1f, %.1f) in, %.1f°", startPose.getX(DistanceUnit.INCH),
                startPose.getY(DistanceUnit.INCH), startPose.getHeading(AngleUnit.DEGREES)));
        if (afterAutonomous != null) {
            telemetry.addLine("初始位姿来源: " + startPoseSource);
        }
        telemetry.addLine("←/→ 调 RPM  LT 吸球  RT 射球  LB 反转  圆圈/B 强制 Limelight 校准（手柄 1、2 都可以）");
        telemetry.addLine("↑/↓ 抬升  三角/Y 切换自瞄（默认开）  方块/X 重置驾驶朝向");
        telemetry.addLine("按下右摇杆 抬起/放下 Flower 铲（默认抬起）");
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

            // Driver forward relative to the robot = the alliance's field direction minus the start
            // heading (the robot starts at the Pinpoint start pose, e.g. where AUTO parked).
            drive.resetYaw((alliance == Alliance.RED
                    ? DriveConstants.RED_DRIVER_FORWARD_FIELD_DEGREES
                    : DriveConstants.BLUE_DRIVER_FORWARD_FIELD_DEGREES)
                    - startPose.getHeading(AngleUnit.DEGREES));
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
            if (deadZoneRumbling) {
                gamepad1.stopRumble();
            }
        }
    }

    @Override
    public void run() {
        updateLoopTime();
        super.run();
        updateDeadZoneRumble();
        telemetry.addData("循环时间", "平均 %.1f ms，最近 2 s 最长 %.1f ms", loopMsAverage,
                Double.isNaN(loopMsShownMax) ? loopMsWindowMax : loopMsShownMax);

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
        telemetry.addData("Flower 铲", "%s (%.2f)",
                flowerScoop.isRaised() ? "抬起" : "放下", flowerScoop.getPosition());
        telemetry.addData("Intake / 通道 实际功率", "%.2f / %.2f", intake.getIntakePower(), intake.getFeedPower());
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
        telemetry.addData("Intake / 通道 实际功率", "%.2f / %.2f", intake.getIntakePower(), intake.getFeedPower());
        telemetry.addData("球挡", gate.isOpen() ? "开" : "关");
        telemetry.addLine("手柄 2  ←/→ 选增益  ↑/↓ 调增益  A 切模式  LB/RB 调手动功率");
        telemetry.addLine("手柄 2 X 切换固定 RPM / 距离插值");
    }

    private void updateLoopTime() {
        long now = System.nanoTime();
        if (lastLoopNs == 0) {
            loopWindowStartNs = now;
        } else {
            double ms = (now - lastLoopNs) / 1e6;
            loopMsAverage = Double.isNaN(loopMsAverage) ? ms : loopMsAverage + 0.05 * (ms - loopMsAverage);
            loopMsWindowMax = Math.max(loopMsWindowMax, ms);
            if (now - loopWindowStartNs >= LOOP_PEAK_WINDOW_NS) {
                loopMsShownMax = loopMsWindowMax;
                loopMsWindowMax = 0;
                loopWindowStartNs = now;
            }
        }
        lastLoopNs = now;
    }

    /**
     * Gamepad 1 rumbles continuously while the turret target is in its dead zone (the light shows red
     * then too). Only the change is sent, so the Driver Station is not sent a rumble every loop.
     */
    private void updateDeadZoneRumble() {
        boolean inDeadZone = turretAimCommand.isInDeadZone();
        if (inDeadZone && !deadZoneRumbling) {
            gamepad1.rumble(1.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
        } else if (!inDeadZone && deadZoneRumbling) {
            gamepad1.stopRumble();
        }
        deadZoneRumbling = inDeadZone;
    }

    private void onPress(BooleanSupplier pressed, Runnable action) {
        new Trigger(() -> isStarted() && pressed.getAsBoolean()).whenActive(action);
    }

    private void scheduleOnPress(BooleanSupplier pressed, Command command) {
        new Trigger(() -> isStarted() && pressed.getAsBoolean()).whenActive(command);
    }
}
