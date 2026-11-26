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

    public static final double START_X_INCHES = 72;
    public static final double START_Y_INCHES = 72;
    public static final double START_HEADING_DEGREES = 180;
    public static final Pose2D START_POSE = new Pose2D(DistanceUnit.INCH, START_X_INCHES,
            START_Y_INCHES, AngleUnit.DEGREES, START_HEADING_DEGREES);
}
