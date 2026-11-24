package org.firstinspires.ftc.teamcode.subsystems.vision;

/** Limelight acquisition and HIVE map geometry. */
public final class VisionConstants {
    private VisionConstants() { }

    public static final String hardwareName = "limelight";
    public static final int bluePipeline = 0;
    public static final int redPipeline = 1;
    public static final int pollRateHz = 50;
    public static final long pipelineInitPollMs = 50;
    public static final long pipelineRetryMs = 1000;
    public static final long maxFrameAgeMs = 200;
    public static final int blueMinTagId = 38;
    public static final int blueMaxTagId = 45;
    public static final int redMinTagId = 30;
    public static final int redMaxTagId = 37;
    public static int minimumTagCount = 2;

    // BIOBUZZ manual V0.9 section 9.6.1: pivot axis 43.95 inches above the mat.
    public static double hivePivotHeightMeters = 43.95 * 0.0254;
    public static double hivePivotXmeters = 0.0;
    public static double maxHiveTiltDegrees = 35.0;
    public static double maxRobotLevelErrorDegrees = 10.0;
    public static double maxFloorErrorMeters = 0.15;
}
