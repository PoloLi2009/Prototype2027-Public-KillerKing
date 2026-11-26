package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

/**
 * Finds {@link DriveConstants#frontStrafeScale}. Hold D-pad left / right to strafe at full power in
 * the robot frame (the same open-loop push as the AutoTune strafe test); release to stop. LB / RB
 * lower / raise the scale. After each strafe it shows how far the heading turned. Pick the scale
 * that keeps both directions near 0 deg and write it into DriveConstants; this test restores the
 * code value when it ends.
 */
@TeleOp(name = "Strafe Compensation Test", group = "Test")
public class StrafeCompensationTest extends LinearOpMode {
    static final double STRAFE_POWER = 1.0;
    static final double SCALE_STEP = 0.02;
    static final double MIN_SCALE = 0.5;
    static final double MAX_SCALE = 1.0;

    @Override
    public void runOpMode() {
        double codeScale = DriveConstants.frontStrafeScale;
        Drive drive = new Drive(hardwareMap);
        boolean lastLower = false;
        boolean lastRaise = false;
        int direction = 0;                 // -1 left, +1 right, 0 stopped
        double startYaw = 0;
        double peak = 0;
        String lastLeft = "-";
        String lastRight = "-";

        telemetry.addLine("Strafe Compensation Test: needs clear space on both sides.");
        telemetry.addData("frontStrafeScale in code", "%.2f", codeScale);
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                boolean lower = gamepad1.left_bumper;
                boolean raise = gamepad1.right_bumper;
                if (lower && !lastLower) DriveConstants.frontStrafeScale -= SCALE_STEP;
                if (raise && !lastRaise) DriveConstants.frontStrafeScale += SCALE_STEP;
                DriveConstants.frontStrafeScale = Range.clip(DriveConstants.frontStrafeScale, MIN_SCALE, MAX_SCALE);
                lastLower = lower;
                lastRaise = raise;

                int wanted = gamepad1.dpad_left ? -1 : gamepad1.dpad_right ? 1 : 0;
                double yaw = drive.getYawDegrees();
                if (wanted != direction) {
                    if (direction != 0) {
                        String result = String.format("turned %+.1f° (peak %.1f°) at scale %.2f",
                                wrap(yaw - startYaw), peak, DriveConstants.frontStrafeScale);
                        if (direction < 0) lastLeft = result;
                        else lastRight = result;
                    }
                    direction = wanted;
                    startYaw = yaw;
                    peak = 0;
                }
                double turned = direction == 0 ? 0 : wrap(yaw - startYaw);
                peak = Math.max(peak, Math.abs(turned));
                drive.driveRobotCentric(0, direction * STRAFE_POWER, 0);   // strafe is right-positive

                telemetry.addLine("Hold D-pad ← / → to strafe at full power. LB / RB: scale -/+ 0.02.");
                telemetry.addData("frontStrafeScale (test)", "%.2f   (code: %.2f)",
                        DriveConstants.frontStrafeScale, codeScale);
                telemetry.addData("Now", direction == 0 ? "stopped" : String.format("strafing %s, turned %+.1f°",
                        direction < 0 ? "left" : "right", turned));
                telemetry.addData("Last left strafe", lastLeft);
                telemetry.addData("Last right strafe", lastRight);
                telemetry.addLine("+ = heading turned left (CCW). Aim for both near 0°.");
                drive.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            drive.stop();
            DriveConstants.frontStrafeScale = codeScale;   // the tuned value only lives in code
        }
    }

    private static double wrap(double degrees) {
        double wrapped = degrees % 360;
        if (wrapped > 180) wrapped -= 360;
        if (wrapped <= -180) wrapped += 360;
        return wrapped;
    }
}
