package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Test for the s3 / s03 servo pair. Both start at 0.5 on INIT. Each press of gamepad 1 D-pad up
 * moves s3 +0.01 and s03 -0.01 (opposite directions); each press of D-pad down does the reverse.
 */
@TeleOp(name = "S3 S03 Servo Pair Test", group = "Quest")
public final class S3ServoPairTest extends LinearOpMode {
    private static final String FIRST_SERVO_NAME = "s3";
    private static final String SECOND_SERVO_NAME = "s03";
    private static final int START_HUNDREDTHS = 50;   // 0.50
    private static final int STEP_HUNDREDTHS = 1;     // 0.01 servo position per press

    private Servo firstServo;
    private Servo secondServo;
    private int hundredths = START_HUNDREDTHS;        // s3 position x 100; s03 mirrors it

    @Override
    public void runOpMode() {
        firstServo = hardwareMap.get(Servo.class, FIRST_SERVO_NAME);
        secondServo = hardwareMap.get(Servo.class, SECOND_SERVO_NAME);
        applyPositions();

        telemetry.addLine("手柄 1 D-pad ↑：s3 +0.01、s03 -0.01；↓：s3 -0.01、s03 +0.01");
        showPositions();
        waitForStart();

        boolean lastUp = false;
        boolean lastDown = false;
        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            if (up && !lastUp) {
                hundredths = Math.min(100, hundredths + STEP_HUNDREDTHS);
            }
            if (down && !lastDown) {
                hundredths = Math.max(0, hundredths - STEP_HUNDREDTHS);
            }
            lastUp = up;
            lastDown = down;

            applyPositions();
            showPositions();
        }
    }

    private void applyPositions() {
        double position = hundredths / 100.0;
        firstServo.setPosition(position);
        secondServo.setPosition(1.0 - position);
    }

    private void showPositions() {
        double position = hundredths / 100.0;
        telemetry.addData("s3 位置", "%.2f", position);
        telemetry.addData("s03 位置", "%.2f", 1.0 - position);
        telemetry.update();
    }
}
