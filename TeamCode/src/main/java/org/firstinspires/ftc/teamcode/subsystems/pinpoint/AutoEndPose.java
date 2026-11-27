package org.firstinspires.ftc.teamcode.subsystems.pinpoint;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Hands an autonomous's last pose to the TeleOp that follows it. OpModes run in the same app process,
 * so a static survives from AUTO to TELEOP; it does not survive an app restart, and then the TeleOp
 * falls back to its preset start pose. Poses are in the TeleOp Pinpoint frame (field center 72 in).
 */
public final class AutoEndPose {
    private AutoEndPose() { }

    // A pose older than this is ignored: the robot may have been moved since. In a match TELEOP's INIT
    // comes during the 8 s transition after AUTO.
    public static final double MAX_AGE_SECONDS = 120;

    private static Pose2D pose;
    private static String source;
    private static long recordedNanos;

    /** Called by an autonomous when it stops (also when the field stops it at 30 s). */
    public static synchronized void record(String autonomousName, Pose2D pinpointPose) {
        pose = pinpointPose;
        source = autonomousName;
        recordedNanos = System.nanoTime();
    }

    /** The pose {@code autonomousName} recorded within MAX_AGE_SECONDS, or null. */
    public static synchronized Pose2D recent(String autonomousName) {
        if (pose == null || !autonomousName.equals(source) || ageSeconds() > MAX_AGE_SECONDS) {
            return null;
        }
        return pose;
    }

    /** Seconds since the last record, or infinity if there is none. */
    public static synchronized double ageSeconds() {
        return pose == null ? Double.POSITIVE_INFINITY : (System.nanoTime() - recordedNanos) / 1e9;
    }
}
