package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.commands.VisionReadCommand;
import org.firstinspires.ftc.teamcode.subsystems.localization.VisionLocalization;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;

public abstract class VisionFusionCommandTest extends CommandOpMode {
    private final LimelightVision.Alliance alliance;
    private Pinpoint pinpoint;
    private LimelightVision vision;
    private VisionReadCommand visionReadCommand;

    protected VisionFusionCommandTest(LimelightVision.Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void initialize() {
        pinpoint = new Pinpoint(hardwareMap);
        Turret turret = new Turret(hardwareMap);
        vision = new LimelightVision(hardwareMap, alliance);
        VisionLocalization localization = new VisionLocalization();
        visionReadCommand = new VisionReadCommand(vision, pinpoint, turret, localization);

        telemetry.addLine("Center the turret before START.");
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

            schedule(visionReadCommand);
            while (opModeIsActive()) {
                run();
            }
        } finally {
            reset();
            if (vision != null) {
                vision.stop();
            }
        }
    }

    @Override
    public void run() {
        super.run();

        pinpoint.addTelemetry(telemetry);
        visionReadCommand.addTelemetry(telemetry);
        telemetry.update();
    }
}
