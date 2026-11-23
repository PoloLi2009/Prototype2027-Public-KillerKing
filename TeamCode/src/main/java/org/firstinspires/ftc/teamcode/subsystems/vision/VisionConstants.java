package org.firstinspires.ftc.teamcode.subsystems.vision;

/** Limelight acquisition settings. Geometry must match the actual uploaded fmaps. */
public final class VisionConstants {
    private VisionConstants() { }
    public static final String hardwareName = "limelight";
    public static final int bluePipeline = 0;
    public static final int redPipeline = 1;
    public static final int pollRateHz = 50;
    public static final long maxFrameAgeMs = 200;

    // The map is field-centered. Initial assumption: Killer King's +X points toward blue HIVE.
    public static final double mapToPinpointYawDegrees = -90;
    public static final double fieldCenterInches = 72;
    public static final double maxSeedDifferenceMeters = 0.75;
    public static final double maxSeedHeadingErrorDegrees = 45;
    public static final double turretSettleSeconds = 0.25;
    // The web camera pose is relative to the floor below the turret pivot.
    public static final double turretPivotForwardMeters = 0;
    public static final double turretPivotLeftMeters = 0;
    // BIOBUZZ manual V0.9 section 9.6.1: pivot axis 43.95 inches above the mat.
    public static double hivePivotHeightMeters = 43.95 * 0.0254;
    public static double hivePivotXmeters = 0.0;
    public static double maxHiveTiltDegrees = 35.0;
    public static double maxRobotLevelErrorDegrees = 10.0;
    public static double maxFloorErrorMeters = 0.15;
    public static int minimumTagCount = 2;
    public static double maxTurretSpeedDegreesPerSecond = 180.0;
    public static double minimumVisionXyStdMeters = 0.08;
    public static double visionDistanceNoiseCoefficient = 0.04;
    public static double visionHeadingStdDegrees = 20.0;
}
