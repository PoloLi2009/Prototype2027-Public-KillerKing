package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.bezier.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.ShooterCommand;
import org.firstinspires.ftc.teamcode.commands.TurretAimCommand;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.gate.Gate;
import org.firstinspires.ftc.teamcode.subsystems.gate.GateConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.localization.VisionLocalization;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionConstants;

import java.util.List;
import java.util.function.Supplier;

/**
 * EXPERIMENTAL red three-tip autonomous for Killer King: the fixed route of {@link Red3TipsAuto}
 * (the stable version, no vision) plus a Limelight tip check, a far recovery and time gates.
 *
 * <p>Pedro drives the route in {@link Red3Tips_1_0}. The turret aims with the TeleOp's
 * {@link TurretAimCommand} (same red targets and travel limit), fed by the Pedro pose, so it
 * pre-aims during each approach. Pickups run the intake with the gate closed; each volley waits for
 * the flywheel and turret, opens the gate, waits the TeleOp pre-open time, then feeds.
 *
 * <p>Tips: the 4 preloads, shot from the start before 01, tip the CELL that starts upward (aimed at
 * (60, 60) from there); 02 + 04 at the far spot fill the CELL that came up (60, 84) for tip 2; 06 +
 * 08 at the near spot go back to (60, 60) for tip 3.
 *
 * <p>Only the 04 volley (tip 2) is checked: the preload volley is assumed to tip, and nothing can
 * rescue a missed last one. The Limelight HIVE tilt tells {@link HiveTipWatcher} whether 04 tipped,
 * and the answer is read when the 05 partner-preload pickup ends, so the check costs no time. If 04
 * did not tip, the partner preloads go back to the far spot to finish tip 2 instead of the near shot.
 *
 * <p>Parking comes first (AUTO PARK only scores in AUTO; a TIP scores the same in TELEOP). Time
 * gates before 06, 07 and the 08 volley skip what no longer fits and park early, and a watchdog
 * cuts any step and parks from where the robot is once the time left only just covers the park.
 */
@Autonomous(name = "Red3Tips1.0 Vision (test)", group = "Auto")
public class Red3TipsVisionAuto extends CommandOpMode {
    // Flywheel targets. Both start at the TeleOp default; tune each from test shots.
    static final double FAR_SHOT_RPM = ShooterConstants.INITIAL_RPM;
    static final double NEAR_SHOT_RPM = ShooterConstants.INITIAL_RPM;
    // 00 preload volley from the start, about 49.5 in from (60, 60): farther than the other shots.
    static final double PRELOAD_SHOT_RPM = ShooterConstants.INITIAL_RPM;
    // The flywheel starts from rest at START, so the preload volley waits longer before it fires on
    // timeout; the other volleys find the flywheel already spinning.
    static final long PRELOAD_READY_TIMEOUT_MS = 3000;
    static final double RPM_TOLERANCE = 100;
    static final long RPM_STABLE_MS = 100;

    // Same power the TeleOp uses for intake, feeding and firing.
    static final double COLLECT_POWER = IntakeConstants.INITIAL_FEED_POWER;
    static final double FEED_POWER = IntakeConstants.INITIAL_FEED_POWER;
    static final long GATE_PRE_OPEN_MS = Math.round(GateConstants.PRE_OPEN_SECONDS * 1000);
    static final long VOLLEY_MS = 900;
    static final long SHOT_READY_TIMEOUT_MS = 1500;
    static final double TURRET_SETTLED_DEGREES = 2.0;

    static final long GARDEN_DWELL_MS = 50;
    static final long FLOWER_DWELL_MS = 1500;
    static final long PARTNER_DWELL_MS = 500;
    static final long PATH_TIMEOUT_MS = 5000;
    static final long BAD_POSE_ABORT_MS = 250;

    // 06 arrives at 90 deg; 07 turns to 180 deg over its first 60% instead of snapping at the start,
    // so the intake is square to the FLOWER before it reaches it.
    static final double SIDE_FLOWER_START_HEADING_DEGREES = 90;
    static final double SIDE_FLOWER_SQUARE_BY = 0.6;

    // TurretAimCommand works in the TeleOp Pinpoint frame (field center 72 in); Pedro's is 70.75 in.
    static final double PEDRO_TO_TURRET_FRAME_INCHES = 72.0 - 70.75;

    // 04 tip check. If the camera saw no HIVE side around the volley, keep the three-tip plan: the
    // recovery would put the partner preloads into the far CELL even if it has already tipped down.
    static final boolean CONTINUE_WHEN_TIP_UNSEEN = true;
    // Time gates, park first. A gate starts its leg only with this much AUTO time left: the leg's
    // typical time (Visualizer speeds, 1.5 s volley) plus what the park watchdog below needs from
    // where the leg ends, so a leg on schedule is never cut halfway. Tune from RobotLog step times.
    static final double AUTO_SECONDS = 30.0;
    static final double NEAR_SHOT_LEG_MIN_SECONDS = 8.0;           // 06 + volley, then park from near
    static final double SIDE_FLOWER_MIN_SECONDS = 10.0;            // 07 + 08 + volley, then 09
    static final double LAST_VOLLEY_MIN_SECONDS = 5.5;             // 08 volley, then 09
    static final double RECOVERY_SIDE_FLOWER_MIN_SECONDS = 12.5;   // 06R + SIDE_FLOWER_MIN_SECONDS
    // Park watchdog: once the time left is no more than the estimated park from the current pose
    // plus this margin, the routine is cut and the robot parks from where it is.
    static final double PARK_MARGIN_SECONDS = 1.0;
    static final double PARK_BASE_SECONDS = 1.0;                   // start-up and settling
    static final double PARK_SPEED_IN_PER_S = 35.0;                // conservative average speed

    private final Red3Tips_1_0 route = new Red3Tips_1_0();
    private final Pose parkPose = route.path909Park().endPose();
    private final HiveTipWatcher tipWatcher = new HiveTipWatcher();
    private Command routine;
    private Command watchdogPark;
    private boolean parking;
    private boolean scoringStopped;
    private Follower follower;
    private Intake intake;
    private Gate gate;
    private Shooter shooter;
    private Turret turret;
    private LimelightVision vision;
    private VisionLocalization localization;
    private ShooterCommand shooterCommand;
    private TurretAimCommand turretAim;
    private String step = "INIT";
    private String shotStatus = "";
    private String plan = "3 tips";
    private String tipCheck = "pending";
    private String lastTimeout;
    private double badPoseSinceMs = -1;
    private final ElapsedTime clock = new ElapsedTime();

    @Override
    public void initialize() {
        follower = Constants.create(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        vision = new LimelightVision(hardwareMap, LimelightVision.Alliance.RED);
        localization = new VisionLocalization();
        turretAim = new TurretAimCommand(this::turretPose, () -> false, turret);
        // Distance RPM stays off (Shooter's default), so the fixed far / near RPM below apply; the
        // distance is passed so it can be switched on with shooter.toggleDistanceRpm().
        shooterCommand = new ShooterCommand(shooter, this::getRuntime, turretAim::getTargetDistanceCm);
        follower.setPose(route.start);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        reset();
        try {
            if (Constants.foresightConfig == null) {
                telemetry.addLine("Pedro is not tuned for Killer King yet.");
                telemetry.addLine("Run AutoTune (Mecanum, Pinpoint, Foresight) and paste the exports into pedro/Constants.java.");
                telemetry.update();
                waitForStart();
                return;
            }

            initialize();
            while (opModeInInit()) {
                vision.preparePipeline(this::opModeInInit);
                telemetry.addData("Red3Tips1.0", "Ready. Place the robot at (%.1f, %.1f) in, %.0f deg.",
                        route.start.x(), route.start.y(), Math.toDegrees(route.start.heading()));
                telemetry.addData("Flywheel preload / far / near", "%.0f / %.0f / %.0f rpm",
                        PRELOAD_SHOT_RPM, FAR_SHOT_RPM, NEAR_SHOT_RPM);
                telemetry.addData("Limelight INIT", "P%d / %s", vision.getExpectedPipeline(), vision.getStatus());
                telemetry.update();
                sleep(VisionConstants.pipelineInitPollMs);
            }
            vision.finishPipelineSetup();
            if (isStopRequested()) return;

            follower.setPose(route.start);
            clock.reset();
            shooter.setTargetRpm(PRELOAD_SHOT_RPM);
            turretAim.toggleAim();                       // enable aiming for the whole routine
            routine = autoRoutine();
            schedule(shooterCommand, turretAim, routine);
            while (opModeIsActive()
                    && (routine.isScheduled() || (watchdogPark != null && watchdogPark.isScheduled()))) {
                run();
            }
        } finally {
            CommandScheduler.getInstance().cancelAll();
            if (intake != null) intake.stop();
            if (gate != null) gate.setOpen(false);
            if (shooter != null) shooter.stop();
            if (follower != null) stopDrive();
            if (vision != null) vision.stop();
            reset();
        }
    }

    private Command autoRoutine() {
        return new SequentialCommandGroup(
                // Tip 1: the 4 preloads from the start, before driving (turret aims at (60, 60)).
                shootPreload("00 Preload / start"),

                collect(followPath("01 Garden", route.path101Garden(), GARDEN_DWELL_MS)),

                setRpm(FAR_SHOT_RPM),
                followPath("02 Garden load / far", route.path202Gardenloadfar(), 0),
                shoot("02 Garden load / far"),

                collect(followPath("03 Far flower", route.path303Farflower(), FLOWER_DWELL_MS)),
                followPath("04 Flower load / far", route.path404Flowerloadfar(), 0),
                shootAndWatchTip("04 Flower load / far"),

                // Both plans pick up the partner preloads next; decide when that pickup ends.
                collect(followPath("05 Partner preloads", route.path505Partnerpreloads(), PARTNER_DWELL_MS)),
                new ConditionalCommand(threeTipFinish(), farRecovery(), this::continueThreeTips));
    }

    /** 04 tipped: partner preloads to the near shot, then the side FLOWER for tip 3. */
    private Command threeTipFinish() {
        return whenTimeLeft("06 near shot", NEAR_SHOT_LEG_MIN_SECONDS,
                new SequentialCommandGroup(
                        setRpm(NEAR_SHOT_RPM),
                        followPath("06 Partner avoid / near", route.path606Partneravoid(), 0),
                        shoot("06 Partner load / near"),
                        sideFlowerLeg()),
                park("09P Partner -> park", this::parkFromHere));
    }

    /**
     * 04 did not tip: take the partner preloads back to 04's far spot, where the turret aims at the
     * still upward CELL, and finish tip 2. Then the side FLOWER if time allows, else park.
     */
    private Command farRecovery() {
        return new SequentialCommandGroup(
                followPath("05R Partner -> far", this::partnerToFarShot, 0),
                shoot("05R Partner load / far"),
                whenTimeLeft("06R near + side flower", RECOVERY_SIDE_FLOWER_MIN_SECONDS,
                        new SequentialCommandGroup(
                                setRpm(NEAR_SHOT_RPM),
                                followPath("06R Far -> near", this::farToNearShot, 0),
                                sideFlowerLeg()),
                        park("09R Far -> park", this::parkFromHere)));
    }

    /**
     * From 06's near spot: 07 + 08 + the last volley, then 09. If time is short it parks from the
     * near spot instead; if only the volley no longer fits, it parks with the balls for TELEOP.
     */
    private Command sideFlowerLeg() {
        return whenTimeLeft("07 side flower", SIDE_FLOWER_MIN_SECONDS,
                new SequentialCommandGroup(
                        collect(followPath("07 Side flower", squareToFlower(route.path707Sideflower()), FLOWER_DWELL_MS)),
                        followPath("08 Flower load / near", route.path808Flowerloadnear(), 0),
                        whenTimeLeft("08 last volley", LAST_VOLLEY_MIN_SECONDS,
                                shoot("08 Flower load / near"), new InstantCommand()),
                        park("09 Park", route.path909Park())),
                park("09N Near -> park", this::parkFromHere));
    }

    /** Runs {@code leg} only with at least {@code minSeconds} of AUTO left, else {@code otherwise}. */
    private Command whenTimeLeft(String name, double minSeconds, Command leg, Command otherwise) {
        return new ConditionalCommand(leg, otherwise, () -> {
            double left = AUTO_SECONDS - clock.seconds();
            boolean go = left >= minSeconds;
            if (!go) plan += " -> skip " + name;
            RobotLog.ii("Red3TipsVision", "Gate %s at %.1f s: %.1f s left, needs %.1f -> %s",
                    name, clock.seconds(), left, minSeconds, go ? "go" : "skip");
            return go;
        });
    }

    /** Stops scoring and drives to the park pose; the park watchdog stays off from here on. */
    private Command park(String name, Path path) {
        return park(name, () -> path);
    }

    private Command park(String name, Supplier<Path> path) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    parking = true;
                    stopScoring();
                }),
                followPath(name, path, 0));
    }

    private boolean continueThreeTips() {
        HiveTipWatcher.Result result = tipWatcher.result();
        boolean keepPlan = result == HiveTipWatcher.Result.TIPPED
                || (result == HiveTipWatcher.Result.UNKNOWN && CONTINUE_WHEN_TIP_UNSEEN);
        tipCheck = result + " (" + tipWatcher.describe() + ")";
        plan = keepPlan ? "3 tips" : "far recovery";
        RobotLog.ii("Red3TipsVision", "04 tip check at %.1f s: %s -> %s", clock.seconds(), tipCheck, plan);
        return keepPlan;
    }

    @Override
    public void run() {
        follower.update();
        if (!poseValid(follower.pose())) {
            // Cut motor power for this loop but keep the active path: follower.stop() would end it,
            // and followPath would then treat the glitch as arriving at the endpoint.
            follower.drivetrain.stop();
            if (badPoseSinceMs < 0) badPoseSinceMs = clock.milliseconds();
            if (clock.milliseconds() - badPoseSinceMs > BAD_POSE_ABORT_MS) {
                RobotLog.ww("Red3TipsVision", "Stopped on %s: Pinpoint pose invalid", step);
                CommandScheduler.getInstance().cancelAll();
            }
        } else {
            badPoseSinceMs = -1;
        }
        super.run();                 // LimelightVision.periodic() reads this loop's frame here
        updateVision();
        checkParkDeadline();
        showTelemetry();
    }

    /** Park watchdog: cut the routine and park from here once the time left only just covers it. */
    private void checkParkDeadline() {
        if (parking || routine == null || !routine.isScheduled()) return;
        Pose pose = follower.pose();
        if (!poseValid(pose)) return;                  // the bad-pose abort above handles this
        double needed = parkSecondsFrom(pose) + PARK_MARGIN_SECONDS;
        double left = AUTO_SECONDS - clock.seconds();
        if (left > needed) return;
        RobotLog.ww("Red3TipsVision", "Park watchdog at %.1f s during %s: %.1f s left, park needs %.1f",
                clock.seconds(), step, left, needed);
        plan += " -> watchdog park";
        routine.cancel();                              // interrupted steps stop the drive and intake
        watchdogPark = park("09W Watchdog park", this::parkFromHere);
        schedule(watchdogPark);
    }

    private double parkSecondsFrom(Pose pose) {
        double distance = Math.hypot(parkPose.x() - pose.x(), parkPose.y() - pose.y());
        return PARK_BASE_SECONDS + distance / PARK_SPEED_IN_PER_S;
    }

    /**
     * Same feed as the TeleOp's VisionReadCommand, but from the Pedro pose (one Pinpoint reader).
     * Pedro's pose is both the odometry and the field pose here. The returned X/Y correction is
     * ignored: the auto never moves Pedro's pose from vision, it only reads the HIVE tilt.
     */
    private void updateVision() {
        double now = System.nanoTime() / 1e9;
        Pose2D pose = turretPose();
        Pose2d odometry = pose == null ? null : new Pose2d(
                pose.getX(DistanceUnit.METER), pose.getY(DistanceUnit.METER),
                new Rotation2d(pose.getHeading(AngleUnit.RADIANS)));
        double estimatedTurret = turret.getEstimatedOffsetDegrees(now);
        localization.update(now, odometry, pose, -estimatedTurret,
                Math.abs(turret.getOffsetDegrees() - estimatedTurret), vision.getNewFrame());
        tipWatcher.update(now, localization.getHiveTiltDegrees(), localization.getHiveMotionState());
    }

    // ---------------------------------------------------------------- heading adjustments

    /** 07: start from 06's heading and face the FLOWER squarely before the intake reaches it. */
    private static Path squareToFlower(Path path) {
        return path.linear(Math.toRadians(SIDE_FLOWER_START_HEADING_DEGREES), Math.toRadians(180),
                SIDE_FLOWER_SQUARE_BY);
    }

    // ---------------------------------------------------------------- recovery paths
    // Built when the step starts, from wherever the robot is. Straight lines between existing route
    // poses; check them in the Pedro Visualizer before relying on them.

    /** 05R: partner preloads back to 04's far shot spot. */
    private Path partnerToFarShot() {
        Pose start = follower.pose();
        Pose far = route.path404Flowerloadfar().endPose();
        return Paths.line(start, far).linear(start, far);
    }

    /** 06R: far shot spot to 06's near shot spot, so 07-09 run unchanged from there. */
    private Path farToNearShot() {
        Pose start = follower.pose();
        Pose near = route.path606Partneravoid().endPose();
        return Paths.line(start, near).linear(start, near);
    }

    /**
     * Early, recovery and watchdog parks, from wherever the robot is. Below the field middle it
     * reuses 09's control points, which swing out past the Side FLOWER before running up the wall;
     * from the far half (the park pose is there) it is a straight line.
     */
    private Path parkFromHere() {
        Pose start = follower.pose();
        Path park09 = route.path909Park();
        if (start.y() < Red3Tips_1_0.FIELD_SIZE_IN / 2 && park09.curve instanceof BezierCurve) {
            List<Vector2D> points = ((BezierCurve) park09.curve).getControlPoints();
            Pose[] through = new Pose[points.size()];
            through[0] = start;
            for (int i = 1; i < points.size(); i++) through[i] = points.get(i).toPose(0);
            return Paths.curve(through).linear(start, parkPose);
        }
        return Paths.line(start, parkPose).linear(start, parkPose);
    }

    // ---------------------------------------------------------------- building blocks

    private Command setRpm(double rpm) {
        return new InstantCommand(() -> shooter.setTargetRpm(rpm));
    }

    /** Runs the intake (gate closed) until {@code drive} finishes, like holding LT in the TeleOp. */
    private Command collect(Command drive) {
        return new ParallelDeadlineGroup(drive, new StartEndCommand(
                () -> {
                    gate.setOpen(false);
                    intake.setPower(COLLECT_POWER);
                },
                intake::stop, intake, gate));
    }

    /** Follows {@code path}, then waits {@code dwellMs} at the endpoint. */
    private Command followPath(String name, Path path, long dwellMs) {
        return followPath(name, () -> path, dwellMs);
    }

    /** Same, but builds the path when the step starts (for paths that begin at the current pose). */
    private Command followPath(String name, Supplier<Path> path, long dwellMs) {
        return new CommandBase() {
            private final ElapsedTime timer = new ElapsedTime();
            private double arrivedMs;
            private boolean done;

            @Override
            public void initialize() {
                timer.reset();
                arrivedMs = -1;
                done = false;
                step = name;
                RobotLog.ii("Red3TipsVision", "%s: start at %.1f s", name, clock.seconds());
                follower.follow(path.get());
            }

            @Override
            public void execute() {
                double now = timer.milliseconds();
                if (arrivedMs < 0) {
                    if (follower.isBusy() && now < PATH_TIMEOUT_MS) return;
                    if (follower.isBusy()) {
                        stopDrive();
                        lastTimeout = name;
                        RobotLog.ww("Red3TipsVision", "%s: path timed out after %.0f ms", name, now);
                    }
                    arrivedMs = now;
                }
                done = now - arrivedMs >= dwellMs;
            }

            @Override
            public boolean isFinished() {
                return done;
            }

            @Override
            public void end(boolean interrupted) {
                if (interrupted) stopDrive();
            }
        };
    }

    private Command shoot(String name) {
        return new ShootCommand(name, false, SHOT_READY_TIMEOUT_MS);
    }

    /** The first volley: the flywheel is still spinning up from rest. */
    private Command shootPreload(String name) {
        return new ShootCommand(name, false, PRELOAD_READY_TIMEOUT_MS);
    }

    /** A volley whose TIP is checked: arms {@link HiveTipWatcher} at the gate, marks the feed end. */
    private Command shootAndWatchTip(String name) {
        return new ShootCommand(name, true, SHOT_READY_TIMEOUT_MS);
    }

    /** Waits for flywheel + turret, opens the gate, then feeds for one volley. */
    private final class ShootCommand extends CommandBase {
        private final String name;
        private final boolean watchTip;
        private final long readyTimeoutMs;
        private final ElapsedTime timer = new ElapsedTime();
        private int phase;                 // 0 aim, 1 gate pre-open, 2 feeding, 3 done
        private double phaseStartMs;
        private double rpmOkSinceMs;

        ShootCommand(String name, boolean watchTip, long readyTimeoutMs) {
            this.name = name;
            this.watchTip = watchTip;
            this.readyTimeoutMs = readyTimeoutMs;
            addRequirements(intake, gate);
        }

        @Override
        public void initialize() {
            timer.reset();
            phase = 0;
            rpmOkSinceMs = -1;
            step = name + " (shoot)";
            intake.stop();
            gate.setOpen(false);
        }

        @Override
        public void execute() {
            double now = timer.milliseconds();
            if (phase == 0) {
                boolean rpmOk = Math.abs(shooter.getTargetRpm() - shooter.getFilteredRpm()) <= RPM_TOLERANCE;
                if (!rpmOk) rpmOkSinceMs = -1;
                else if (rpmOkSinceMs < 0) rpmOkSinceMs = now;
                boolean rpmStable = rpmOkSinceMs >= 0 && now - rpmOkSinceMs >= RPM_STABLE_MS;
                boolean turretSettled = Math.abs(turret.getEstimatedOffsetDegrees(System.nanoTime() / 1e9)
                        - turret.getOffsetDegrees()) <= TURRET_SETTLED_DEGREES;
                shotStatus = String.format("waiting: flywheel %s, turret %s",
                        rpmStable ? "ok" : "spinning", turretSettled ? "ok" : "moving");
                if ((rpmStable && turretSettled) || now >= readyTimeoutMs) {
                    if (!(rpmStable && turretSettled)) {
                        RobotLog.ww("Red3TipsVision", "%s: fired on timeout (%s)", name, shotStatus);
                    }
                    gate.setOpen(true);
                    if (watchTip) tipWatcher.arm(System.nanoTime() / 1e9);
                    phase = 1;
                    phaseStartMs = now;
                }
            } else if (phase == 1 && now - phaseStartMs >= GATE_PRE_OPEN_MS) {
                intake.setPower(FEED_POWER);
                phase = 2;
                phaseStartMs = now;
                shotStatus = "feeding";
            } else if (phase == 2 && now - phaseStartMs >= VOLLEY_MS) {
                if (watchTip) tipWatcher.volleyEnded(System.nanoTime() / 1e9);
                phase = 3;
                shotStatus = "done";
            }
        }

        @Override
        public boolean isFinished() {
            return phase == 3;
        }

        @Override
        public void end(boolean interrupted) {
            intake.stop();
            gate.setOpen(false);
        }
    }

    // ---------------------------------------------------------------- helpers

    private void stopScoring() {
        if (scoringStopped) return;                           // toggleAim would turn aim back on
        scoringStopped = true;
        if (turretAim.isScheduled()) turretAim.toggleAim();   // disabled aim re-centers the turret
        shooterCommand.cancel();                              // ShooterCommand.end() stops the flywheel
    }

    private Pose2D turretPose() {
        Pose pose = follower.pose();
        if (!poseValid(pose)) return null;
        return new Pose2D(DistanceUnit.INCH,
                pose.x() + PEDRO_TO_TURRET_FRAME_INCHES, pose.y() + PEDRO_TO_TURRET_FRAME_INCHES,
                AngleUnit.RADIANS, pose.heading());
    }

    private static boolean poseValid(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    private void stopDrive() {
        follower.stop();
        follower.drivetrain.stop();
    }

    private void showTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("Step", step);
        telemetry.addData("Time", "%.1f s", clock.seconds());
        if (lastTimeout != null) telemetry.addData("Last path timeout", lastTimeout);
        telemetry.addData("Pose (in, deg)", "%.1f, %.1f, %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Shot", shotStatus);
        telemetry.addData("Plan", plan);
        telemetry.addData("04 tip check", tipCheck);
        double tilt = localization.getHiveTiltDegrees();
        telemetry.addData("HIVE", Double.isFinite(tilt)
                ? String.format("%.1f° / %s", tilt, localization.getHiveMotionState()) : "not seen");
        telemetry.addData("Limelight", vision.getStatus());
        shooter.addTelemetry(telemetry);
        turretAim.addTelemetry(telemetry);
        telemetry.update();
    }
}
