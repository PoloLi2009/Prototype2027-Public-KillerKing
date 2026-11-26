package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.PinpointConstants;

@TeleOp(name = "Pinpoint Direction Test", group = "Quest")
public final class PinpointDirectionTest extends CommandOpMode {
    private Drive drive;
    private Pinpoint pinpoint;
    private Pose2d reference;

    @Override
    public void initialize() {
        drive = new Drive(hardwareMap);
        drive.setDefaultCommand(new DriveCommand(drive, gamepad1));
        pinpoint = new Pinpoint(hardwareMap);
        new Trigger(() -> gamepad1.a).whenActive(() -> reference = pinpoint.getRawOdometry());
    }

    @Override
    public void runOpMode() {
        reset();
        try {
            initialize();
            while (opModeInInit()) {
                pinpoint.periodic();
                addInstructions();
                pinpoint.addTelemetry(telemetry);
                telemetry.update();
                sleep(50);
            }
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
        }
    }

    @Override
    public void run() {
        super.run();
        addInstructions();
        pinpoint.addTelemetry(telemetry);

        Pose2d current = pinpoint.getRawOdometry();
        if (current != null) {
            if (reference == null) {
                reference = current;
            }
            Translation2d fieldDelta = current.getTranslation().minus(reference.getTranslation());
            Transform2d robotDelta = current.minus(reference);
            telemetry.addData("场地 ΔX / ΔY", "%+.2f / %+.2f in",
                    DistanceUnit.INCH.fromMeters(fieldDelta.getX()),
                    DistanceUnit.INCH.fromMeters(fieldDelta.getY()));
            telemetry.addData("标记时车体方向 Δ前 / Δ左", "%+.2f / %+.2f in",
                    DistanceUnit.INCH.fromMeters(robotDelta.getTranslation().getX()),
                    DistanceUnit.INCH.fromMeters(robotDelta.getTranslation().getY()));
            telemetry.addData("航向变化", "%+.1f°", robotDelta.getRotation().getDegrees());
        }
        telemetry.update();
    }

    private void addInstructions() {
        telemetry.addLine("手柄 1 左摇杆平移，右摇杆自转。A 标记测量起点。");
        telemetry.addLine("蓝方站位：Intake 朝红方启动，左移预期 Y−，后移预期 X+。");
        telemetry.addData("Pinpoint 编码器 X / Y", "%s / %s",
                PinpointConstants.X_ENCODER_DIRECTION, PinpointConstants.Y_ENCODER_DIRECTION);
    }
}
