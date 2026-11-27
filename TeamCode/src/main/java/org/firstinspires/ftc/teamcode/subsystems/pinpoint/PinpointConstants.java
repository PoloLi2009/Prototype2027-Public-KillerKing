package org.firstinspires.ftc.teamcode.subsystems.pinpoint;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public final class PinpointConstants {
    private PinpointConstants() { }

    public static final String DEVICE_NAME = "od";
    public static final double X_OFFSET_MM = -48.0;
    public static final double Y_OFFSET_MM = -125.0;
    // The longitudinal pod must report positive robot-relative X for forward motion.
    public static final GoBildaPinpointDriver.EncoderDirection X_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    // The lateral pod must report positive robot-relative Y for leftward motion.
    public static final GoBildaPinpointDriver.EncoderDirection Y_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // Default start: field center, intake toward the red alliance wall. Used by the Limelight tuning
    // TeleOps and the tests; the Red / Blue TeleOps use the AUTO park poses below.
    public static final double START_X_INCHES = 72;
    public static final double START_Y_INCHES = 72;
    public static final double START_HEADING_DEGREES = 180;
    public static final Pose2D START_POSE = new Pose2D(DistanceUnit.INCH, START_X_INCHES,
            START_Y_INCHES, AngleUnit.DEGREES, START_HEADING_DEGREES);

    // Red / Blue TeleOps start where Red2Tips1.0 / Blue2Tips1.0 park: the end of 07 Park, red
    // (11.9513, 94.5223) and blue (129.5487, 46.9777) in Pedro inches, plus the Pedro-to-Pinpoint
    // shift (Pedro's field center is 70.75 in, Pinpoint's 72, so +1.25 in on X and Y). Headings are
    // the same in both frames: 07's reverse tangent at its end, 0.1337 deg (red) and 180.1337 (blue).
    // If the park point changes, update these.
    public static final Pose2D RED_TELEOP_START_POSE = new Pose2D(DistanceUnit.INCH, 13.2013, 95.7723,
            AngleUnit.DEGREES, 0.1337);
    public static final Pose2D BLUE_TELEOP_START_POSE = new Pose2D(DistanceUnit.INCH, 130.7987, 48.2277,
            AngleUnit.DEGREES, 180.1337);

    // The TeleOp "Red After 3Tips" starts where Red3Tips2.0 ends, right after pushing the partner into
    // the LOADING ZONE: Red3Tips_2_0.end, Pedro (9.5, 88.5) at 270 deg (intake toward the audience, the
    // back against the partner), plus the same +1.25 in shift. It uses this only when Red3Tips2.0 did
    // not hand over its measured end pose (AutoEndPose: app restarted, or more than 2 minutes since).
    // If Red3Tips_2_0.end changes, update this.
    public static final Pose2D RED_AFTER_3TIPS_START_POSE = new Pose2D(DistanceUnit.INCH, 10.75, 89.75,
            AngleUnit.DEGREES, 270);
}
