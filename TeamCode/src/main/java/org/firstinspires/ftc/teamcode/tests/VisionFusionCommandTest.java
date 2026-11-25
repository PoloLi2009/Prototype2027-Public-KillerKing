package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;

public abstract class VisionFusionCommandTest extends CommandOpMode {
    private final LimelightVision.Alliance alliance;
    private Drive drive;
    private Pinpoint pinpoint;
    private LimelightVision vision;

    protected VisionFusionCommandTest(LimelightVision.Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap);
        drive.setDefaultCommand(new DriveCommand(drive, gamepad1));
        pinpoint = new Pinpoint(hardwareMap);
        new Turret(hardwareMap); // Keep the camera at the turret's calibrated center.
        vision = new LimelightVision(hardwareMap, alliance);

        telemetry.addLine("Gamepad 1: left stick translates, right stick X turns.");
        telemetry.addData("Required camera pipeline", "%s P%d (selected in INIT)",
                alliance, vision.getExpectedPipeline());
        telemetry.update();
    }

    @Override
    public void runOpMode() {
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
            schedule(drive.getDefaultCommand());
            while (opModeIsActive()) {
                run();
            }
        } finally {
            reset();
            if (drive != null) {
                drive.stop();
            }
            if (vision != null) {
                vision.stop();
            }
        }
    }

    @Override
    public void run() {
        super.run();

        pinpoint.addTelemetry(telemetry);
        telemetry.addData("Limelight", "P%d / %s", vision.getExpectedPipeline(), vision.getStatus());
        LimelightVision.Frame frame = vision.getLastRawFrame();
        if (frame == null) {
            telemetry.addData("Limelight raw MT1 (fmap)", "Waiting for pose");
        } else {
            Position position = frame.rawPose.getPosition();
            telemetry.addData("Limelight raw MT1 X/Y/Yaw (fmap)", "%.2f / %.2f in / %.1f°",
                    DistanceUnit.INCH.fromUnit(position.unit, position.x),
                    DistanceUnit.INCH.fromUnit(position.unit, position.y),
                    frame.rawPose.getOrientation().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Raw MT1 last frame", "%.0f ms ago / %d tags",
                    (System.nanoTime() - frame.approximateCaptureNanos) / 1e6, frame.tagCount);
        }
        telemetry.addData("Gamepad 1 right stick", "X %+.2f / click %b",
                gamepad1.right_stick_x, gamepad1.right_stick_button);
        drive.addTelemetry(telemetry);
        telemetry.update();
    }
}
