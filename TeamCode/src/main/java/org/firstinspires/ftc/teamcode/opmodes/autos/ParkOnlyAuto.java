package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Park-only autonomous: does nothing else, waits START_DELAY_SECONDS after START, then drives one
 * straight line from {@link #start} to {@link #park} and holds there until AUTO ends. The park point
 * is the Red2Tips1.0 / Blue2Tips1.0 park (end of 07 Park, within 0.03 in), so the default "Red" /
 * "Blue" TeleOp's preset start pose (PinpointConstants.RED/BLUE_TELEOP_START_POSE) fits it.
 * Red and blue differ only in their poses: blue is red rotated 180 deg about the field center
 * (x' = 141.5 - x, y' = 141.5 - y, heading + 180).
 */
public abstract class ParkOnlyAuto extends LinearOpMode {
    static final double START_DELAY_SECONDS = 25;
    static final long PATH_TIMEOUT_MS = 4000;
    static final long BAD_POSE_ABORT_MS = 250;

    private final String name;
    private final Pose start;
    private final Pose park;

    protected ParkOnlyAuto(String name, Pose start, Pose park) {
        this.name = name;
        this.start = start;
        this.park = park;
    }

    @Override
    public void runOpMode() {
        if (Constants.foresightConfig == null) {
            telemetry.addLine("Pedro is not tuned for Killer King yet.");
            telemetry.update();
            waitForStart();
            return;
        }
        // The Foresight config is static; clear a speed cap left by a cut FLOWER approach in another auto.
        Constants.foresightConfig.maxVelocityConstraint.set(ForesightConfig.Constraint.NONE);
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(start);

        while (opModeInInit()) {
            telemetry.addData(name, "Ready. Place the robot at (%.2f, %.2f) in, %.0f deg.",
                    start.x(), start.y(), Math.toDegrees(start.heading()));
            telemetry.addData("Park", "(%.2f, %.2f) in, %.0f deg, %.0f s after START",
                    park.x(), park.y(), Math.toDegrees(park.heading()), START_DELAY_SECONDS);
            telemetry.update();
            sleep(50);
        }
        if (isStopRequested()) return;

        follower.setPose(start);
        ElapsedTime clock = new ElapsedTime();
        ElapsedTime pathTimer = new ElapsedTime();
        String step = "waiting";
        boolean started = false;
        boolean finished = false;
        double badPoseSinceMs = -1;
        try {
            while (opModeIsActive()) {
                follower.update();
                Pose pose = follower.pose();
                boolean valid = poseValid(pose);

                if (!started && clock.seconds() >= START_DELAY_SECONDS) {
                    Path path = Paths.line(start, park).constant(park);
                    follower.follow(path);
                    pathTimer.reset();
                    started = true;
                    step = "driving";
                    RobotLog.ii(name, "park: start at %.1f s", clock.seconds());
                }

                if (!valid) {
                    follower.drivetrain.stop();
                    if (badPoseSinceMs < 0) badPoseSinceMs = clock.milliseconds();
                    if (clock.milliseconds() - badPoseSinceMs > BAD_POSE_ABORT_MS) {
                        RobotLog.ww(name, "Stopped on %s: Pinpoint pose invalid", step);
                        break;
                    }
                } else {
                    badPoseSinceMs = -1;
                }

                if (started && !finished) {
                    if (!follower.isBusy()) {
                        finished = true;
                        step = "parked";
                        RobotLog.ii(name, "park: arrived at %.1f s, %.2f in from the end", clock.seconds(),
                                valid ? Math.hypot(pose.x() - park.x(), pose.y() - park.y()) : Double.NaN);
                    } else if (pathTimer.milliseconds() >= PATH_TIMEOUT_MS) {
                        finished = true;
                        step = "park timed out";
                        follower.stop();
                        follower.drivetrain.stop();
                        RobotLog.ww(name, "park: path timed out, %.2f in from the end",
                                valid ? Math.hypot(pose.x() - park.x(), pose.y() - park.y()) : Double.NaN);
                    }
                }

                telemetry.addData("Step", step);
                telemetry.addData("Time", "%.1f s", clock.seconds());
                if (!started) telemetry.addData("Park starts in", "%.1f s", START_DELAY_SECONDS - clock.seconds());
                if (valid) {
                    telemetry.addData("Pose (in, deg)", "%.2f, %.2f, %.1f",
                            pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                }
                telemetry.update();
            }
        } finally {
            follower.stop();
            follower.drivetrain.stop();
        }
    }

    private static boolean poseValid(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }
}
