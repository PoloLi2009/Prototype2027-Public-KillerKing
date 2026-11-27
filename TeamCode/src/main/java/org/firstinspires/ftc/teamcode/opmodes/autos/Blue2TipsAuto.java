package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.curves.Curve;
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
import org.firstinspires.ftc.teamcode.subsystems.flowerscoop.FlowerScoop;
import org.firstinspires.ftc.teamcode.subsystems.gate.Gate;
import org.firstinspires.ftc.teamcode.subsystems.gate.GateConstants;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * Blue alliance two-tip autonomous for Killer King: the fallback with miss tolerance. Fixed route, no
 * vision, no branches, no partner preloads. Built from {@link Blue3TipsAuto} (same building blocks),
 * which stays unchanged.
 *
 * <p>Blue copy of {@link Red2TipsAuto}, same program: the route is {@link Blue2Tips_1_0} (the red
 * route rotated 180 deg about the field center) and the turret aims at the blue HIVE. The turret
 * offsets and distances are the same as red's. Keep the two files in step when changing either.
 *
 * <p>Pedro drives the route in {@link Blue2Tips_1_0}. The turret aims with the TeleOp's
 * {@link TurretAimCommand} (same blue targets and travel limit), fed by the Pedro pose, so it
 * pre-aims during each approach. Pickups run the intake with the gate closed, and the two FLOWER
 * pickups (03, 05) also lower the flower scoop and slow down for their last inches; each volley
 * waits for the flywheel and turret, opens the gate, waits the TeleOp pre-open time, then feeds.
 * Each volley has its own flywheel RPM.
 *
 * <p>Tips: the 4 preloads, shot from the start before 01, tip the CELL that starts upward (aimed at
 * (84, 84) from there). Then every pickup goes to the FAR CELL that came up, (84, 60): Garden (02),
 * Far flower (04) and the near FLOWER (06), 12 balls for the 8 that tip 2 needs. Each step's start
 * time goes to RobotLog.
 */
@Autonomous(name = "Blue2Tips1.0", group = "Auto")
public class Blue2TipsAuto extends CommandOpMode {
    // Flywheel target for each volley (robot center to target); tune each from test shots at its spot.
    // The three far shots overshot at the TeleOp default 3375, so they take the TeleOp distance table
    // (ShooterConstants.DISTANCE_RPM_POINTS, which holds 3177 below its first point, 109.4 cm) at their
    // distance, rounded to the 25 RPM step. The preload is not a far shot and stays at the default.
    // 00 shot high at 3375, 3350 and 3325: 150 RPM under the TeleOp default, 3225.
    static final double PRELOAD_SHOT_RPM = ShooterConstants.INITIAL_RPM - 150; // 00: 49.5 in (126 cm) to (84, 84)
    static final double GARDEN_SHOT_RPM = 3175;                           // 02: 40.7 in (103 cm) to (84, 60)
    // 04 moved 10 cm toward the alliance wall: 45.9 in (116.5 cm), the table gives 3222.
    static final double FAR_FLOWER_SHOT_RPM = 3225;                       // 04: 45.9 in (116.5 cm) to (84, 60)
    // 06 shoots from (92.48, 18.65) at 340 deg (turning from 0 along the path): the turret needs
    // -146.4 deg, 13.6 deg inside its +/-160 deg travel. At 0 deg it would have needed -166.4.
    static final double NEAR_FLOWER_SHOT_RPM = 3175;                      // 06: 41.3 in (105 cm) to (84, 60)
    static final double RPM_TOLERANCE = 100;
    static final long RPM_STABLE_MS = 100;

    static final long GATE_PRE_OPEN_MS = Math.round(GateConstants.PRE_OPEN_SECONDS * 1000);
    // Volleys feed at IntakeConstants.AUTO_FIRE_FEED_MOTOR_POWER (0.6). The feed time was 900 ms at
    // the TeleOp's 0.8 and is scaled by 0.8 / 0.6 so all 4 balls still leave; trim it from tests.
    static final long VOLLEY_MS = 1200;
    static final long SHOT_READY_TIMEOUT_MS = 1500;
    // The flywheel starts from rest at START, so the preload volley waits longer before it fires on
    // timeout; the other volleys find the flywheel already spinning.
    static final long PRELOAD_READY_TIMEOUT_MS = 3000;
    static final double TURRET_SETTLED_DEGREES = 2.0;
    // Before every volley the intake and feed motors run with the gate closed for at least
    // PRE_FEED_MS, pushing a ball caught between the intake and the channel up to the gate (after a
    // Garden pickup one stayed there and the volley fired only 3). They start PRE_FEED_START_INCHES
    // before the shot spot, so most of the second overlaps the drive and the flywheel wait; starting
    // earlier would risk picking up a stray ball on the way (G407: at most 4).
    static final long PRE_FEED_MS = 1000;
    static final double PRE_FEED_START_INCHES = 12;

    // Garden and partner-preload pickups: collect PICKUP_DWELL_MS at the point, back straight off
    // PICKUP_BACK_OFF_INCHES (5 cm), drive back to the same spot and collect PICKUP_DWELL_MS again.
    // Picking up in one go kept missing balls.
    static final long PICKUP_DWELL_MS = 500;
    static final double PICKUP_BACK_OFF_INCHES = 5.0 / 2.54;
    // A pickup point counts as reached once the robot stops within PICKUP_STALL_INCHES of it: walls
    // or balls can stop it short, and with the general 1.5 in the Garden path once waited out the
    // 5 s path timeout. The back-off and return then use the spot where it stopped.
    static final double PICKUP_STALL_INCHES = 4;
    // At a FLOWER the robot stays at most FLOWER_MAX_MS, counted from reaching it, then moves on even
    // if Pedro has not finished the path (the intake can jam on the FLOWER and stop short). Reaching
    // = within FLOWER_CONTACT_INCHES of the FLOWER point (the FLOWER sticks out about 3.8 in from the
    // wall, so the intake touches it about 4 in before the point), the path finished, or stopped
    // anywhere in the slow approach.
    static final long FLOWER_MAX_MS = 1500;
    static final double FLOWER_CONTACT_INCHES = 5;
    // The flower scoop stays lowered after a FLOWER pickup and is raised once the robot is
    // FLOWER_SCOOP_CLEAR_INCHES from where it left the FLOWER, or on reaching the next shot spot if
    // that comes first: raised right at the FLOWER, the scoop caught on it. The intake and feed keep
    // running (gate closed) until the scoop is up. Far flower: the scoop stays down through 04 and
    // is raised while the 04 volley fires, when the gate opens (raised on the way out it jammed).
    static final double FLOWER_SCOOP_CLEAR_INCHES = 6;
    // After the Garden pickup the intake and feed keep running (gate closed) for this long while 02
    // drives off, so balls still in the intake mouth are pulled in and all 4 stay.
    static final long GARDEN_EXTRA_INTAKE_MS = 1000;
    // Intake height (s0, larger = higher) once a pickup ends and the robot drives off: lowered from
    // the pickup height IntakeConstants.LIFT_POSITION (0.56). Each pickup sets 0.56 again when it
    // starts. The Garden back-off and return are part of the pickup, so they stay at 0.56.
    static final double TRAVEL_LIFT_POSITION = 0.5;
    static final long PATH_TIMEOUT_MS = 5000;
    static final long BAD_POSE_ABORT_MS = 250;

    // 03 and 05 drive their last FLOWER_APPROACH_INCHES at no more than this speed, so the intake
    // meets the FLOWER gently (Pedro's maxVelocityConstraint, applied to that piece only).
    // 12 in since the Far flower point moved 5 cm deeper, so the same stretch before the FLOWER is slow.
    static final double FLOWER_APPROACH_INCHES = 12;
    static final double FLOWER_APPROACH_SPEED_IN_PER_S = 35;   // was 20; raised for a firmer push at the Far flower
    // A FLOWER pickup also counts as arrived once the robot stops within FLOWER_STALL_INCHES of the
    // point. The robot stops against the Far flower 7.0-8.5 in short of the exported point, which is
    // 9.0-10.9 in short of the point 5 cm deeper: with the old 10 in window (Blue2Tips1.0 at 04:00 and
    // 04:08 on 2026-11-27) it waited out the 5 s path timeout there.
    static final double FLOWER_STALL_INCHES = 14;
    // The Far flower point sits about 0.46 in past wall contact, which Pedro may never reach.
    // A path counts as arrived once the robot is within END_STALL_DISTANCE of its end and has moved
    // less than END_STALL_MOVE over END_STALL_MS; it keeps pressing in during the dwell.
    static final double END_STALL_DISTANCE_INCHES = 1.5;
    static final double END_STALL_MOVE_INCHES = 0.2;
    static final long END_STALL_MS = 200;

    // TurretAimCommand works in the TeleOp Pinpoint frame (field center 72 in); Pedro's is 70.75 in.
    static final double PEDRO_TO_TURRET_FRAME_INCHES = 72.0 - 70.75;

    private final Blue2Tips_1_0 route = new Blue2Tips_1_0();
    private Follower follower;
    private Intake intake;
    private Gate gate;
    private FlowerScoop flowerScoop;
    private Shooter shooter;
    private Turret turret;
    private ShooterCommand shooterCommand;
    private TurretAimCommand turretAim;
    private Command routine;
    private String step = "INIT";
    private String shotStatus = "";
    private String lastTimeout;
    private double badPoseSinceMs = -1;
    private double preFeedStartMs = -1;   // clock ms when the current volley's pre-feed started
    private final ElapsedTime clock = new ElapsedTime();

    @Override
    public void initialize() {
        // The Foresight config is static and outlives OpModes; make sure no speed cap is left from a
        // run that was cut inside a slow FLOWER approach (Pedro reverts it on follow() / stop()).
        Constants.foresightConfig.maxVelocityConstraint.set(ForesightConfig.Constraint.NONE);
        follower = Constants.create(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        flowerScoop = new FlowerScoop(hardwareMap);   // raised at INIT, as in the TeleOp
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        turretAim = new TurretAimCommand(this::turretPose, () -> true, turret);
        // Distance RPM stays off (Shooter's default), so the fixed RPM above apply.
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
                telemetry.addData("Blue2Tips1.0", "Ready. Place the robot at (%.1f, %.1f) in, %.0f deg.",
                        route.start.x(), route.start.y(), Math.toDegrees(route.start.heading()));
                telemetry.addData("Flywheel 00 / 02 / 04 / 06", "%.0f / %.0f / %.0f / %.0f rpm",
                        PRELOAD_SHOT_RPM, GARDEN_SHOT_RPM, FAR_FLOWER_SHOT_RPM, NEAR_FLOWER_SHOT_RPM);
                telemetry.update();
                sleep(50);
            }
            if (isStopRequested()) return;

            follower.setPose(route.start);
            clock.reset();
            shooter.setTargetRpm(PRELOAD_SHOT_RPM);
            // Set, not toggled: TurretAimCommand's default changed to on, and a toggle then turned it off.
            turretAim.setAimEnabled(true);
            routine = autoRoutine();
            schedule(shooterCommand, turretAim, routine);
            while (opModeIsActive() && routine.isScheduled()) {
                run();
            }
        } finally {
            cancelCommands();
            if (intake != null) intake.stop();
            if (gate != null) gate.setOpen(false);
            if (flowerScoop != null) flowerScoop.raise();
            if (shooter != null) shooter.stop();
            if (follower != null) stopDrive();
            reset();
        }
    }

    private Command autoRoutine() {
        return new SequentialCommandGroup(
                // Tip 1: the 4 preloads from the start, before driving (turret aims at (84, 84)).
                shootPreload("00 Preload / start"),

                // The Garden RPM is set before the pickup, so the intake runs on without a gap into 02.
                setRpm(GARDEN_SHOT_RPM),
                collect(pickupWithBackOff("01 Garden", route.path101Garden())),
                withIntakeFor(GARDEN_EXTRA_INTAKE_MS,
                        followToShot("02 Garden load / far", route.path202Gardenloadfar())),
                shoot("02 Garden load / far"),

                setRpm(FAR_FLOWER_SHOT_RPM),
                collectFromFlower(followIntoFlower("03 Far flower", slowFlowerApproach(route.path303Farflower()))),
                // The scoop stays down through 04 and is raised as the 04 volley fires.
                withIntake(followToShot("04 Flower load / far", route.path404Flowerloadfar())),
                shoot("04 Flower load / far", this::raiseScoopWhileFiring),

                // The near (Side) FLOWER, carried back to a far spot: still (84, 60), like 02 and 04.
                setRpm(NEAR_FLOWER_SHOT_RPM),
                collectFromFlower(followIntoFlower("05 Near flower", slowFlowerApproach(route.path505NearFlower()))),
                raiseScoopOnceClear(followToShot("06 Near flower load / far", route.path606Flowerloadnear())),
                shoot("06 Near flower load / far"),

                new InstantCommand(this::stopScoring),
                followPath("07 Park", route.path707Park(), 0));
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
                RobotLog.ww("Blue2Tips1.0", "Stopped on %s: Pinpoint pose invalid", step);
                cancelCommands();
            }
        } else {
            badPoseSinceMs = -1;
        }
        super.run();
        showTelemetry();
    }

    // ---------------------------------------------------------------- path adjustments

    private static Path slowFlowerApproach(Path path) {
        return slowFinish(path, FLOWER_APPROACH_INCHES, FLOWER_APPROACH_SPEED_IN_PER_S);
    }

    /**
     * {@code path} with its last {@code inches} capped at {@code speedInPerS}. The Bezier curve is split
     * there (de Casteljau, same shape) into two pieces of one continuous path; the end piece carries
     * Foresight's maxVelocityConstraint, which Pedro applies only while that piece is followed and
     * reverts on the next follow() or stop(). Headings are sampled from {@code path} at the split.
     */
    static Path slowFinish(Path path, double inches, double speedInPerS) {
        Curve curve = path.curve;
        if (!(curve instanceof BezierCurve) || curve.length() <= inches) {
            return path.with(Constants.foresightConfig.maxVelocityConstraint.at(speedInPerS));
        }
        double lo = 0;
        double hi = 1;                                  // remainingDistance falls from length() to 0
        for (int i = 0; i < 40; i++) {
            double mid = (lo + hi) / 2;
            if (curve.remainingDistance(mid) > inches) lo = mid; else hi = mid;
        }
        double split = (lo + hi) / 2;

        // de Casteljau: the first point of every level is a control point of the head piece, the last
        // point of every level (in reverse) one of the tail piece.
        List<Vector2D> level = new ArrayList<>(((BezierCurve) curve).getControlPoints());
        List<Vector2D> head = new ArrayList<>();
        List<Vector2D> tail = new ArrayList<>();
        while (!level.isEmpty()) {
            head.add(level.get(0));
            tail.add(0, level.get(level.size() - 1));
            List<Vector2D> next = new ArrayList<>();
            for (int i = 0; i + 1 < level.size(); i++) {
                next.add(level.get(i).times(1 - split).plus(level.get(i + 1).times(split)));
            }
            level = next;
        }
        Path fast = Paths.curve(head.toArray(new Vector2D[0]))
                .linear(path.heading(0), path.heading(split));
        Path slow = Paths.curve(tail.toArray(new Vector2D[0]))
                .linear(path.heading(split), path.heading(1))
                .with(Constants.foresightConfig.maxVelocityConstraint.at(speedInPerS));
        return Paths.path(fast, slow);
    }

    // ---------------------------------------------------------------- building blocks

    private Command setRpm(double rpm) {
        return new InstantCommand(() -> shooter.setTargetRpm(rpm));
    }

    /**
     * Runs the intake (gate closed, at the pickup height IntakeConstants.LIFT_POSITION) until
     * {@code drive} finishes, then stops it, like holding LT in the TeleOp, and lowers the intake to
     * TRAVEL_LIFT_POSITION: the pickup is over and the next step drives off in the same loop.
     */
    private Command collect(Command drive) {
        return new ParallelDeadlineGroup(latch(drive), new StartEndCommand(
                () -> {
                    gate.setOpen(false);
                    intake.setLiftPosition(IntakeConstants.LIFT_POSITION);
                    intake.setPowers(IntakeConstants.INTAKE_MOTOR_POWER, IntakeConstants.FEED_MOTOR_POWER);
                },
                () -> {
                    intake.stop();
                    intake.setLiftPosition(TRAVEL_LIFT_POSITION);
                    RobotLog.ii("Blue2Tips1.0", "%s: pickup done, intake lowered to %.2f at %.1f s",
                            step, TRAVEL_LIFT_POSITION, clock.seconds());
                },
                intake, gate));
    }

    /**
     * {@link #collect} at a FLOWER with the flower scoop lowered. The scoop stays down when the pickup
     * ends; the next step keeps the intake on and raises the scoop once the robot is clear of the FLOWER
     * ({@link #raiseScoopOnceClear}), and stopping the routine raises it too ({@link #cancelCommands},
     * finally).
     */
    private Command collectFromFlower(Command drive) {
        return new SequentialCommandGroup(new InstantCommand(flowerScoop::lower, flowerScoop), collect(drive));
    }

    /** Raises the flower scoop as a volley fires (passed to {@link #shoot(String, Runnable)}). */
    private void raiseScoopWhileFiring() {
        flowerScoop.raise();
        RobotLog.ii("Blue2Tips1.0", "%s: flower scoop raised as the volley fires at %.1f s", step, clock.seconds());
    }

    /**
     * Runs {@code drive} with the intake and feed on (gate closed) the whole way; they stop when it
     * ends unless the volley's pre-feed has started and is using them.
     */
    private Command withIntake(Command drive) {
        return withIntakeFor(Long.MAX_VALUE, drive);
    }

    /**
     * Runs {@code drive} (the path away from a FLOWER) and raises the flower scoop once the robot is
     * FLOWER_SCOOP_CLEAR_INCHES from where the step started, or when {@code drive} ends if that is first.
     * The intake and feed run (gate closed) until then and stop with the raise, unless the volley's
     * pre-feed has started and is using them.
     */
    private Command raiseScoopOnceClear(Command drive) {
        return new ParallelDeadlineGroup(latch(drive), new CommandBase() {
            private Pose from;

            @Override
            public void initialize() {
                from = follower.pose();
                gate.setOpen(false);
                intake.setPowers(IntakeConstants.INTAKE_MOTOR_POWER, IntakeConstants.FEED_MOTOR_POWER);
            }

            @Override
            public boolean isFinished() {
                Pose pose = follower.pose();
                return !poseValid(from)
                        || (poseValid(pose) && distance(pose, from) >= FLOWER_SCOOP_CLEAR_INCHES);
            }

            @Override
            public void end(boolean interrupted) {
                flowerScoop.raise();
                if (preFeedStartMs < 0) intake.stop();
                Pose pose = follower.pose();
                RobotLog.ii("Blue2Tips1.0", "%s: flower scoop raised %.1f in from the FLOWER at %.1f s", step,
                        poseValid(from) && poseValid(pose) ? distance(pose, from) : Double.NaN, clock.seconds());
            }
        });
    }

    /**
     * Runs {@code drive} with the intake and feed on (gate closed) for its first {@code ms}, then stops
     * them, unless the volley's pre-feed has started by then and is using them.
     */
    private Command withIntakeFor(long ms, Command drive) {
        return new ParallelDeadlineGroup(latch(drive), new CommandBase() {
            private final ElapsedTime timer = new ElapsedTime();

            @Override
            public void initialize() {
                timer.reset();
                gate.setOpen(false);
                intake.setPowers(IntakeConstants.INTAKE_MOTOR_POWER, IntakeConstants.FEED_MOTOR_POWER);
            }

            @Override
            public boolean isFinished() {
                return timer.milliseconds() >= ms;
            }

            @Override
            public void end(boolean interrupted) {
                if (preFeedStartMs < 0) intake.stop();
                RobotLog.ii("Blue2Tips1.0", "%s: intake kept on %.0f ms after the pickup", step, timer.milliseconds());
            }
        });
    }

    /**
     * Runs {@code command} and stays finished once it has finished. Every deadline goes through this:
     * FTCLib 2.1.1's ParallelDeadlineGroup ends its deadline and only then asks deadline.isFinished(),
     * and a SequentialCommandGroup answers false after end() (its index is reset to -1), so a group
     * deadline such as {@link #pickupWithBackOff} never let the pickup end: the intake kept running
     * and the robot stopped for the rest of AUTO. (A ParallelRaceGroup instead ends it twice and
     * throws IndexOutOfBoundsException.)
     */
    private static Command latch(Command command) {
        return new CommandBase() {
            private boolean done;

            @Override
            public void initialize() {
                done = false;
                command.initialize();
            }

            @Override
            public void execute() {
                if (done) return;
                command.execute();
                if (command.isFinished()) {
                    command.end(false);
                    done = true;
                }
            }

            @Override
            public boolean isFinished() {
                return done;
            }

            @Override
            public void end(boolean interrupted) {
                if (!done) command.end(true);
            }
        };
    }

    /**
     * Follows {@code path}, then waits {@code dwellMs} at the endpoint. A robot stopped just short of
     * the end (pressed against a wall) counts as arrived; Pedro keeps pushing during the dwell.
     */
    private Command followPath(String name, Path path, long dwellMs) {
        return followPath(name, () -> path, dwellMs, 0, END_STALL_DISTANCE_INCHES, 0);
    }

    /** Same, but builds the path when the step starts (for paths that begin at the current pose). */
    private Command followPath(String name, Supplier<Path> path, long dwellMs) {
        return followPath(name, path, dwellMs, 0, END_STALL_DISTANCE_INCHES, 0);
    }

    /**
     * Follows {@code path} to a shot spot and starts the pre-volley feed (see PRE_FEED_MS) once the robot
     * is within PRE_FEED_START_INCHES of it.
     */
    private Command followToShot(String name, Path path) {
        return followPath(name, () -> path, 0, 0, END_STALL_DISTANCE_INCHES, PRE_FEED_START_INCHES);
    }

    /**
     * Follows {@code path} into a FLOWER and moves on {@link #FLOWER_MAX_MS} after reaching it, whether
     * or not Pedro has finished (see FLOWER_CONTACT_INCHES for what counts as reaching it).
     */
    private Command followIntoFlower(String name, Path path) {
        return followPath(name, () -> path, FLOWER_MAX_MS, FLOWER_CONTACT_INCHES, FLOWER_STALL_INCHES, 0);
    }

    /**
     * Follows {@code path} and finishes {@code dwellMs} after arriving. Arriving is the first of: Pedro
     * finished the path; the robot within {@code arriveWithinInches} of the end; the robot stopped
     * (moved less than END_STALL_MOVE over END_STALL_MS) within {@code stallWithinInches} of the end;
     * or the path timeout. Only the timeout stops the drive: otherwise Pedro keeps pushing toward the
     * end until the next step's follow() replaces the path. Within {@code preFeedWithinInches} of the
     * end (0 = never) it starts the pre-volley feed.
     */
    private Command followPath(String name, Supplier<Path> path, long dwellMs, double arriveWithinInches,
                               double stallWithinInches, double preFeedWithinInches) {
        return new CommandBase() {
            private final ElapsedTime timer = new ElapsedTime();
            private Pose end;
            private double arrivedMs;
            private boolean done;
            private Pose stallStart;
            private double stallStartMs;

            @Override
            public void initialize() {
                timer.reset();
                arrivedMs = -1;
                done = false;
                stallStart = null;
                step = name;
                RobotLog.ii("Blue2Tips1.0", "%s: start at %.1f s", name, clock.seconds());
                Path current = path.get();
                end = current.endPose();
                follower.follow(current);
            }

            @Override
            public void execute() {
                double now = timer.milliseconds();
                if (arrivedMs < 0) {
                    Pose pose = follower.pose();
                    double toEnd = poseValid(pose) ? distance(pose, end) : Double.POSITIVE_INFINITY;
                    if (toEnd <= preFeedWithinInches) startPreFeed();
                    boolean busy = follower.isBusy();
                    boolean close = busy && toEnd <= arriveWithinInches;
                    boolean stalled = busy && stalled(now, pose, toEnd);
                    if (busy && !close && !stalled && now < PATH_TIMEOUT_MS) return;
                    if (close) {
                        RobotLog.ii("Blue2Tips1.0", "%s: reached at %.1f s, %.1f in from the end; leaving in %d ms",
                                name, clock.seconds(), toEnd, dwellMs);
                    } else if (stalled) {
                        RobotLog.ii("Blue2Tips1.0", "%s: stopped %.2f in short of the end, counted as arrived",
                                name, toEnd);
                    } else if (busy) {
                        stopDrive();
                        lastTimeout = name;
                        RobotLog.ww("Blue2Tips1.0", "%s: path timed out after %.0f ms, %.1f in from the end",
                                name, now, toEnd);
                    }
                    // A stall counts from when the robot stopped, not from when it was detected.
                    arrivedMs = stalled && !close ? stallStartMs : now;
                }
                done = now - arrivedMs >= dwellMs;
            }

            /** Within stallWithinInches of the end and moved less than END_STALL_MOVE over END_STALL_MS. */
            private boolean stalled(double now, Pose pose, double toEnd) {
                if (toEnd > stallWithinInches) {
                    stallStart = null;
                    return false;
                }
                if (stallStart == null || distance(pose, stallStart) > END_STALL_MOVE_INCHES) {
                    stallStart = pose;
                    stallStartMs = now;
                    return false;
                }
                return now - stallStartMs >= END_STALL_MS;
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

    /**
     * Garden and partner-preload pickup: drives {@code path}, collects PICKUP_DWELL_MS there, backs
     * straight off PICKUP_BACK_OFF_INCHES (5 cm, away from the intake side), drives back to the spot it
     * stopped at and collects PICKUP_DWELL_MS again. Wrap it in collect() so the intake runs throughout.
     */
    private Command pickupWithBackOff(String name, Path path) {
        Pose[] spot = new Pose[1];
        return new SequentialCommandGroup(
                followPath(name, () -> path, PICKUP_DWELL_MS, 0, PICKUP_STALL_INCHES, 0),
                new InstantCommand(() -> {
                    Pose pose = follower.pose();
                    spot[0] = poseValid(pose) ? pose : path.endPose();
                }),
                followPath(name + " / back off", () -> Paths.line(spot[0], backOff(spot[0])).constant(spot[0]), 0),
                followPath(name + " / return", () -> {
                    Pose here = follower.pose();
                    // A zero-length line is not followable; if the robot did not move, return from the
                    // planned back-off point instead.
                    Pose from = poseValid(here) && distance(here, spot[0]) > 0.25 ? here : backOff(spot[0]);
                    return Paths.line(from, spot[0]).constant(spot[0]);
                }, PICKUP_DWELL_MS));
    }

    /** {@code pose} moved PICKUP_BACK_OFF_INCHES straight backward (opposite its heading). */
    private static Pose backOff(Pose pose) {
        return new Pose(pose.x() - PICKUP_BACK_OFF_INCHES * Math.cos(pose.heading()),
                pose.y() - PICKUP_BACK_OFF_INCHES * Math.sin(pose.heading()), pose.heading());
    }

    private Command shoot(String name) {
        return new ShootCommand(name, SHOT_READY_TIMEOUT_MS, null);
    }

    /** Same, and runs {@code onFire} when the volley fires (the gate opens). */
    private Command shoot(String name, Runnable onFire) {
        return new ShootCommand(name, SHOT_READY_TIMEOUT_MS, onFire);
    }

    /** The first volley: the flywheel is still spinning up from rest. */
    private Command shootPreload(String name) {
        return new ShootCommand(name, PRELOAD_READY_TIMEOUT_MS, null);
    }

    /**
     * Runs the intake and feed motors with the gate closed, once per volley (see PRE_FEED_MS). Called by
     * the approach to a shot spot or, at the latest, when the volley starts; ShootCommand.end() resets it.
     */
    private void startPreFeed() {
        if (preFeedStartMs >= 0) return;
        preFeedStartMs = clock.milliseconds();
        gate.setOpen(false);
        intake.setPowers(IntakeConstants.INTAKE_MOTOR_POWER, IntakeConstants.FEED_MOTOR_POWER);
    }

    /**
     * Runs the pre-volley feed for at least PRE_FEED_MS and waits for flywheel + turret, opens the gate
     * (motors paused for the TeleOp pre-open time), then feeds for one volley.
     */
    private final class ShootCommand extends CommandBase {
        private final String name;
        private final long readyTimeoutMs;
        private final Runnable onFire;     // run when the gate opens, or null
        private final ElapsedTime timer = new ElapsedTime();
        private int phase;                 // 0 aim, 1 gate pre-open, 2 feeding, 3 done
        private double phaseStartMs;
        private double rpmOkSinceMs;

        ShootCommand(String name, long readyTimeoutMs, Runnable onFire) {
            this.name = name;
            this.readyTimeoutMs = readyTimeoutMs;
            this.onFire = onFire;
            addRequirements(intake, gate);
        }

        @Override
        public void initialize() {
            timer.reset();
            phase = 0;
            rpmOkSinceMs = -1;
            step = name + " (shoot)";
            RobotLog.ii("Blue2Tips1.0", "%s: shoot at %.1f s", name, clock.seconds());
            startPreFeed();                       // no-op if the approach already started it
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
                boolean preFed = clock.milliseconds() - preFeedStartMs >= PRE_FEED_MS;
                shotStatus = String.format("waiting: flywheel %s, turret %s, pre-feed %s",
                        rpmStable ? "ok" : "spinning", turretSettled ? "ok" : "moving", preFed ? "ok" : "running");
                if (preFed && ((rpmStable && turretSettled) || now >= readyTimeoutMs)) {
                    if (!(rpmStable && turretSettled)) {
                        RobotLog.ww("Blue2Tips1.0", "%s: fired on timeout (%s)", name, shotStatus);
                    }
                    intake.stop();                // like the TeleOp: motors pause while the gate opens
                    gate.setOpen(true);
                    if (onFire != null) onFire.run();
                    phase = 1;
                    phaseStartMs = now;
                }
            } else if (phase == 1 && now - phaseStartMs >= GATE_PRE_OPEN_MS) {
                intake.setPowers(IntakeConstants.INTAKE_MOTOR_POWER, IntakeConstants.AUTO_FIRE_FEED_MOTOR_POWER);
                phase = 2;
                phaseStartMs = now;
                shotStatus = "feeding";
            } else if (phase == 2 && now - phaseStartMs >= VOLLEY_MS) {
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
            preFeedStartMs = -1;                  // the next volley runs its own pre-feed
        }
    }

    // ---------------------------------------------------------------- helpers

    /**
     * Cancels the commands this OpMode scheduled, running each end(). Not cancelAll(): FTCLib 2.1.1
     * removes entries from the map it is iterating, which throws ConcurrentModificationException
     * whenever two or more commands are scheduled (always the case here).
     */
    private void cancelCommands() {
        CommandScheduler.getInstance().cancel(routine, shooterCommand, turretAim);
        if (flowerScoop != null) flowerScoop.raise();   // a FLOWER pickup leaves it down until the next step
    }

    private void stopScoring() {
        turretAim.setAimEnabled(false);                       // disabled aim re-centers the turret
        shooterCommand.cancel();                              // ShooterCommand.end() stops the flywheel
    }

    private Pose2D turretPose() {
        Pose pose = follower.pose();
        if (!poseValid(pose)) return null;
        return new Pose2D(DistanceUnit.INCH,
                pose.x() + PEDRO_TO_TURRET_FRAME_INCHES, pose.y() + PEDRO_TO_TURRET_FRAME_INCHES,
                AngleUnit.RADIANS, pose.heading());
    }

    private static double distance(Pose a, Pose b) {
        return Math.hypot(a.x() - b.x(), a.y() - b.y());
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
        telemetry.addData("Flower scoop", "%s (%.2f)",
                flowerScoop.isRaised() ? "raised" : "lowered", flowerScoop.getPosition());
        shooter.addTelemetry(telemetry);
        turretAim.addTelemetry(telemetry);
        telemetry.update();
    }
}
