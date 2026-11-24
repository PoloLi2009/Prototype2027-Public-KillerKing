package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.VisionReadCommand;
import org.firstinspires.ftc.teamcode.subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.localization.VisionLocalization;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;

@TeleOp(name = "Vision Fusion Command Test", group = "Test")
@SuppressWarnings("unused") // FTC discovers this class through @TeleOp.
public final class VisionFusionCommandTest extends CommandOpMode {
    private Pinpoint pinpoint;
    private LimelightVision vision;
    private VisionReadCommand visionReadCommand;
    private boolean previousX;

    @Override
    public void initialize() {
        pinpoint = new Pinpoint(hardwareMap);
        Turret turret = new Turret(hardwareMap);
        vision = new LimelightVision(hardwareMap);
        VisionLocalization localization = new VisionLocalization();
        visionReadCommand = new VisionReadCommand(vision, pinpoint, turret, localization);

        telemetry.addLine("Center the turret before START. Press X to switch alliance maps.");
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        reset();
        try {
            initialize();
            waitForStart();
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
        boolean x = gamepad1.x;
        if (x && !previousX) {
            vision.toggleAlliance();
        }
        previousX = x;

        super.run();

        pinpoint.addTelemetry(telemetry);
        visionReadCommand.addTelemetry(telemetry);
        telemetry.update();
    }
}
