package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

/** Acquires each MT1 frame once. Raw fmap coordinates are not robot field coordinates yet. */
public class LimelightVision extends SubsystemBase {
    public static final class Frame {
        public final Pose3D rawPose;
        public final long approximateCaptureNanos;
        public final double cameraTimestamp;
        public final int pipeline;
        public final int tagCount;
        public final double averageDistanceMeters;
        Frame(Pose3D pose, long captureNanos, double timestamp, int pipeline, int tagCount,
              double averageDistanceMeters) {
            this.rawPose = pose;
            this.approximateCaptureNanos = captureNanos;
            this.cameraTimestamp = timestamp;
            this.pipeline = pipeline;
            this.tagCount = tagCount;
            this.averageDistanceMeters = averageDistanceMeters;
        }
    }

    private final Limelight3A camera;
    private final ExecutorService pipelineExecutor;
    private boolean blueAlliance = true;
    private double lastTimestamp = Double.NaN;
    private Future<?> pendingPipelineSwitch;
    private long lastPipelineRequestNanos;
    private Frame frame;
    private String status = "Waiting for frame";

    public LimelightVision(HardwareMap hardwareMap) {
        Limelight3A device = null;
        try {
            device = hardwareMap.tryGet(Limelight3A.class, VisionConstants.hardwareName);
            if (device != null) {
                device.setPollRateHz(VisionConstants.pollRateHz);
                device.start();
            } else {
                status = "Not configured";
            }
        } catch (RuntimeException error) {
            RobotLog.ee("KillerKingVision", error, "Could not start Limelight");
            device = null;
            status = "Initialization failed";
        }
        camera = device;
        pipelineExecutor = camera == null ? null : Executors.newSingleThreadExecutor();
    }

    public void toggleAlliance() {
        blueAlliance = !blueAlliance;
        frame = null;
        lastTimestamp = Double.NaN;
        if (camera != null) {
            status = "Waiting for new pipeline frame";
        }
    }

    @Override
    public void periodic() {
        frame = null;
        if (camera == null) return;
        if (!camera.isConnected()) { status = "Disconnected"; return; }
        LLResult result = camera.getLatestResult();
        if (result == null) { status = "No result"; return; }
        int expectedPipeline = getExpectedPipeline();
        if (result.getPipelineIndex() != expectedPipeline) {
            requestPipeline(expectedPipeline);
            status = "Waiting for pipeline";
            return;
        }
        if (!result.isValid()) { status = "No valid result"; return; }
        double timestamp = result.getTimestamp();
        if (!Double.isFinite(timestamp) || timestamp <= 0) {
            status = "Invalid camera timestamp"; return;
        }
        if (timestamp == lastTimestamp) { status = "No new frame"; return; }
        lastTimestamp = timestamp;
        long age = result.getStaleness();
        double latency = result.getCaptureLatency() + result.getTargetingLatency();
        if (age < 0 || age > VisionConstants.maxFrameAgeMs
                || !Double.isFinite(latency) || latency < 0
                || age + latency > VisionConstants.maxFrameAgeMs) {
            status = "Frame too old"; return;
        }
        Pose3D pose = result.getBotpose();
        if (result.getBotposeTagCount() < VisionConstants.minimumTagCount || pose == null
                || pose.getPosition() == null || pose.getPosition().unit == null
                || pose.getOrientation() == null) {
            status = "No MT1 pose"; return;
        }
        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
            status = "No tag IDs"; return;
        }
        // Red and blue HIVE structures tilt independently; never apply one correction to both.
        StringBuilder unknownTagIds = new StringBuilder();
        int minTagId = blueAlliance ? VisionConstants.blueMinTagId : VisionConstants.redMinTagId;
        int maxTagId = blueAlliance ? VisionConstants.blueMaxTagId : VisionConstants.redMaxTagId;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (id < minTagId || id > maxTagId) {
                if (unknownTagIds.length() > 0) unknownTagIds.append(",");
                unknownTagIds.append(id);
            }
        }
        if (unknownTagIds.length() > 0) {
            status = "Unexpected HIVE tag ID(s): " + unknownTagIds;
            return;
        }
        double distance = result.getBotposeAvgDist();
        if (!Double.isFinite(distance) || distance <= 0) {
            status = "Invalid target distance"; return;
        }
        // Camera time has a different clock origin; estimate host capture time from age + latency.
        long captureNanos = System.nanoTime() - (long) ((age + latency) * 1e6);
        frame = new Frame(pose, captureNanos, timestamp,
                result.getPipelineIndex(), result.getBotposeTagCount(), distance);
        status = "New MT1 frame";
    }

    private void requestPipeline(int pipeline) {
        long now = System.nanoTime();
        long retryNanos = TimeUnit.SECONDS.toNanos(VisionConstants.pipelineRetrySeconds);
        if (pendingPipelineSwitch != null && (!pendingPipelineSwitch.isDone()
                || now - lastPipelineRequestNanos < retryNanos)) {
            return;
        }
        lastPipelineRequestNanos = now;
        // The SDK switches pipelines through a synchronous HTTP request.
        pendingPipelineSwitch = pipelineExecutor.submit(() -> camera.pipelineSwitch(pipeline));
    }

    /** Returns only a new frame from the most recent periodic call, otherwise null. */
    public Frame getNewFrame() { return frame; }
    public boolean isBlueAlliance() { return blueAlliance; }
    public int getExpectedPipeline() {
        return blueAlliance ? VisionConstants.bluePipeline : VisionConstants.redPipeline;
    }
    public String getStatus() { return status; }
    public void stop() {
        if (camera != null) {
            pipelineExecutor.shutdownNow();
            camera.stop();
        }
        frame = null;
    }
}
