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
import java.util.function.BooleanSupplier;

/** Acquires each MT1 frame once. Raw fmap coordinates are not robot field coordinates yet. */
public class LimelightVision extends SubsystemBase {
    public enum Alliance { BLUE, RED }

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
    private final Alliance alliance;
    private ExecutorService pipelineExecutor;
    private Future<?> pipelineRequest;
    private long lastPipelineRequestNanos;
    private boolean pipelineSetupFinished;
    private double lastTimestamp = Double.NaN;
    private Frame frame;
    private Frame lastRawFrame;
    private String status = "Waiting for frame";

    public LimelightVision(HardwareMap hardwareMap, Alliance alliance) {
        this.alliance = alliance;
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
    }

    public void preparePipeline(BooleanSupplier inInit) {
        if (camera == null || pipelineSetupFinished || !inInit.getAsBoolean()) {
            return;
        }
        if (!camera.isConnected()) {
            status = "Waiting for Limelight connection";
            return;
        }

        int desiredPipeline = getExpectedPipeline();
        LLResult result = camera.getLatestResult();
        if (result != null && result.getPipelineIndex() == desiredPipeline) {
            status = "Pipeline P" + desiredPipeline + " ready";
            return;
        }

        status = "Selecting P" + desiredPipeline + "; waiting for camera confirmation";
        long now = System.nanoTime();
        if (pipelineRequest != null && (!pipelineRequest.isDone()
                || now - lastPipelineRequestNanos < TimeUnit.MILLISECONDS.toNanos(VisionConstants.pipelineRetryMs))) {
            return;
        }
        if (pipelineExecutor == null) {
            pipelineExecutor = Executors.newSingleThreadExecutor();
        }
        lastPipelineRequestNanos = now;
        // The SDK's HTTP request can block. Keep it off the OpMode thread and start it only in INIT.
        pipelineRequest = pipelineExecutor.submit(() -> {
            if (inInit.getAsBoolean() && !Thread.currentThread().isInterrupted()) {
                camera.pipelineSwitch(desiredPipeline);
            }
        });
    }

    public void finishPipelineSetup() {
        pipelineSetupFinished = true;
        if (pipelineExecutor != null) {
            pipelineExecutor.shutdownNow();
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
        int actualPipeline = result.getPipelineIndex();
        if (actualPipeline != expectedPipeline) {
            status = "Wrong pipeline: camera P" + actualPipeline + ", expected P" + expectedPipeline;
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
        int tagCount = result.getBotposeTagCount();
        if (pose == null || pose.getPosition() == null || pose.getPosition().unit == null
                || pose.getOrientation() == null) {
            status = "No MT1 pose"; return;
        }
        double distance = result.getBotposeAvgDist();
        // Camera time has a different clock origin; estimate host capture time from age + latency.
        long captureNanos = System.nanoTime() - (long) ((age + latency) * 1e6);
        Frame rawFrame = new Frame(pose, captureNanos, timestamp, actualPipeline, tagCount, distance);
        if (tagCount > 0) {
            lastRawFrame = rawFrame;
        }
        if (tagCount < VisionConstants.minimumTagCount) {
            status = "Insufficient HIVE tags"; return;
        }
        if (result.getFiducialResults() == null || result.getFiducialResults().isEmpty()) {
            status = "No tag IDs"; return;
        }
        // Red and blue HIVE structures tilt independently; never apply one correction to both.
        StringBuilder unknownTagIds = new StringBuilder();
        int minTagId = isBlueAlliance() ? VisionConstants.blueMinTagId : VisionConstants.redMinTagId;
        int maxTagId = isBlueAlliance() ? VisionConstants.blueMaxTagId : VisionConstants.redMaxTagId;
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
        if (!Double.isFinite(distance) || distance <= 0) {
            status = "Invalid target distance"; return;
        }
        frame = rawFrame;
        status = "New MT1 frame";
    }

    /** Returns only a new frame from the most recent periodic call, otherwise null. */
    public Frame getNewFrame() { return frame; }

    /** Returns the last fresh MT1 pose before application-specific HIVE checks. */
    public Frame getLastRawFrame() { return lastRawFrame; }

    public boolean isBlueAlliance() { return alliance == Alliance.BLUE; }
    public int getExpectedPipeline() {
        return isBlueAlliance() ? VisionConstants.bluePipeline : VisionConstants.redPipeline;
    }
    public String getStatus() { return status; }
    public void stop() {
        finishPipelineSetup();
        if (camera != null) {
            camera.stop();
        }
        frame = null;
        lastRawFrame = null;
    }
}
