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
import com.pedropathing.paths.Path;
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
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.AutoEndPose;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

import java.util.function.Supplier;

/**
 * Red alliance three-tip autonomous 2.0 for Killer King. Fixed route, no vision. Built from the
 * latest two-tip, {@link Red2TipsAuto} (same building blocks and the same 00-04), with the
 * three-tip's partner preloads and Side flower for tip 3. {@link Red3TipsAuto} (1.0) and the two-tip
 * stay unchanged.
 *
 * <p>Route ({@link Red3Tips_2_0}): 00 the 4 preloads from the start into the near CELL (60, 60), tip 1;
 * 01 Garden; 02 to the foot of the Far flower and fire the 4 Garden balls there; 03 turn, go into the
 * Far flower empty, collect its 4 and fire them from inside: 8 into (60, 84), tip 2; 04-05 partner
 * preloads, 06 near volley; 07 Side flower, 08 near volley into (60, 60), tip 3; 09 push the partner
 * into the LOADING ZONE with the robot's back and stop there, both robots in the zone. The end pose
 * goes to {@link AutoEndPose} for the TeleOp "Red After 3Tips".
 *
 * <p>What differs from the two-tip's blocks: the motors that feed the gate count as pre-fed from
 * when they last started, so a volley after a FLOWER (they never stop) fires as soon as the flywheel
 * and turret are ready; the pre-feed before the other volleys starts 40 in out instead of 12; a FLOWER
 * step has its own short time limit; the Garden and partner back-off count as done 0.3 in from their
 * target. There is no time gate: 07 and 08 always run (the user's choice, 2026-11-27); if AUTO ends
 * before the push is done, the TeleOp still gets the pose where the robot stopped.
 * The robot only goes into a FLOWER empty: the Garden balls are fired in front of the Far flower
 * first, so it never holds them and the FLOWER's at once. The Far flower's balls are fired with the
 * robot pressed into the FLOWER; its scoop is raised on the way out, FAR_SCOOP_CLEAR_INCHES from where
 * the robot stood. The Side flower's is raised as the 08 volley fires, about 10 in from where the robot
 * stood.
 */
@Autonomous(name = Red3Tips2Auto.NAME, group = "Auto")
public class Red3Tips2Auto extends CommandOpMode {
    /** OpMode name, RobotLog tag and {@link AutoEndPose} source. */
    public static final String NAME = "Red3Tips2.0";
    private static final String TAG = NAME;

    // Flywheel target for each volley (robot center to target). 00 is the two-tip's spot and value. The
    // others take the two-tip's value for about the same distance: 3175 at its 103.5 and 105 cm shots,
    // 3225 at 116.5 cm. Inside the Far flower the robot is 108-118 cm from the target, depending on where
    // the scoop stops it. Every volley logs its distance; check them with test shots.
    static final double PRELOAD_SHOT_RPM = ShooterConstants.INITIAL_RPM - 150; // 00: 125.7 cm to (60, 60)
    static final double GARDEN_SHOT_RPM = 3175;                           // 02: 101.0 cm to (60, 84)
    static final double FAR_FLOWER_SHOT_RPM = 3225;                       // 03: 108-118 cm to (60, 84)
    // 06 and 08 are closer than any tested auto shot (the nearest is 103.5 cm at 3175); start there.
    static final double PARTNER_SHOT_RPM = 3175;                          // 06: 88.8 cm to (60, 60)
    static final double SIDE_FLOWER_SHOT_RPM = 3175;                      // 08: 92.4 cm to (60, 60)
    static final double RPM_TOLERANCE = 100;
    static final long RPM_STABLE_MS = 100;

    static final long GATE_PRE_OPEN_MS = Math.round(GateConstants.PRE_OPEN_SECONDS * 1000);
    // Volleys feed at IntakeConstants.AUTO_FIRE_FEED_MOTOR_POWER (0.6) for this long: 1200 ms in the
    // two-tip, cut to 1000 ms for time (user, 2026-11-27). Raise it again if the 4th ball stays in.
    static final long VOLLEY_MS = 1000;
    static final long SHOT_READY_TIMEOUT_MS = 1500;
    // The flywheel starts from rest at START, so the preload volley waits longer before it fires on
    // timeout; the other volleys find the flywheel already spinning.
    static final long PRELOAD_READY_TIMEOUT_MS = 3000;
    static final double TURRET_SETTLED_DEGREES = 2.0;
    // A volley after a drive also waits until the robot is within SHOT_SPOT_INCHES of the shot spot and
    // still (moved less than END_STALL_MOVE and turned less than END_STALL_TURN) for SHOT_STILL_MS.
    // Pedro reports a path as finished 100 ms after the robot's projection onto it reaches 97.5 %
    // (Foresight's timeoutConstraint and parametricTConstraint), wherever the robot is: on the
    // 2026-11-27 07:00 run the 06 volley fired that way before the robot was at the spot, and none of
    // the 4 balls went in. The robot not getting there by SHOT_READY_TIMEOUT_MS fires anyway (logged).
    static final double SHOT_SPOT_INCHES = 1.5;
    static final long SHOT_STILL_MS = 100;
    // Before every volley the intake and feed motors must have run with the gate closed for at least
    // PRE_FEED_MS without a stop, pushing a ball caught between the intake and the channel up to the
    // gate. After a FLOWER they run from the pickup on. Otherwise they start PRE_FEED_START_INCHES
    // before the shot spot (about the last second of the drive), so the wait is over on arrival.
    static final long PRE_FEED_MS = 1000;
    static final double PRE_FEED_START_INCHES = 40;

    // Garden pickup: collect PICKUP_DWELL_MS at the point, back straight off PICKUP_BACK_OFF_INCHES
    // (5 cm), drive back to the same spot and collect PICKUP_DWELL_MS again. Picking up in one go kept
    // missing balls. The back-off and the return count as done PICKUP_NEAR_INCHES from their target,
    // without waiting for Pedro to settle on it; Pedro keeps driving to it meanwhile. The dwell was
    // 500 ms in the two-tip, cut for time (user, 2026-11-27).
    static final long PICKUP_DWELL_MS = 350;
    // Partner-preload pickup: one go, PARTNER_DWELL_MS at the point, no back-off (user, 2026-11-27).
    static final long PARTNER_DWELL_MS = 500;
    static final double PICKUP_BACK_OFF_INCHES = 5.0 / 2.54;
    static final double PICKUP_NEAR_INCHES = 0.3;
    // A pickup point counts as reached once the robot stops within PICKUP_STALL_INCHES of it: walls,
    // balls or the partner can stop it short. The Garden back-off and return then use the spot where
    // it stopped.
    static final double PICKUP_STALL_INCHES = 4;
    // After a Garden or partner-preload pickup the intake and feed keep running (gate closed) for this
    // long while the robot drives off, so balls still in the intake mouth are pulled in.
    static final long PICKUP_EXTRA_INTAKE_MS = 1000;
    // Intake height (s0, larger = higher) once a pickup ends and the robot drives off: lowered from
    // the pickup height IntakeConstants.LIFT_POSITION (0.56). Each pickup sets 0.56 again when it
    // starts. The back-off and return are part of the pickup, so they stay at 0.56.
    static final double TRAVEL_LIFT_POSITION = 0.5;

    // At a FLOWER the robot stays FLOWER_MAX_MS, counted from reaching it, then moves on even if Pedro
    // has not finished the path (the intake can jam on the FLOWER and stop short). Reaching = within
    // FLOWER_CONTACT_INCHES of the FLOWER point, the path finished, or stopped within
    // FLOWER_STALL_INCHES of the point (the lowered scoop stops the robot 5-8 in short of it). The
    // whole step, drive included, never takes longer than FLOWER_STEP_MAX_MS: it has waited out the
    // 5 s path timeout at the Far flower before. FLOWER_MAX_MS was 1500 in 1.0 and the two-tip, then
    // 1000, now 750 for time (user, 2026-11-27); raise it first if a FLOWER leaves a ball behind.
    static final long FLOWER_MAX_MS = 750;
    static final long FLOWER_STEP_MAX_MS = 3000;
    static final double FLOWER_CONTACT_INCHES = 5;
    // The Far flower point is 2.4 in deeper than the two-tip's, so the robot stops 9.4-13.3 in short of
    // it (7.1-10.9 in short of the old point).
    static final double FLOWER_STALL_INCHES = 16;
    // At the Side flower the robot stops 5.4-7.1 in short; its step starts with the 24 in turn-in, so
    // the window stays small enough not to catch a pause there.
    static final double SIDE_FLOWER_STALL_INCHES = 10;
    // The Far flower scoop is raised on the way out to the partner preloads, once the robot is this far
    // from where it stood at the FLOWER (raised 6 in out it caught on it; 11-16 in out it did not).
    static final double FAR_SCOOP_CLEAR_INCHES = 10;
    // 02 ends 4 in before the lowered scoop meets the Far flower, facing 270 deg, and 03 faces 90. Pedro
    // turns and drives at the same time, so after the Garden volley the robot first turns on the spot
    // (2.3 in from the FLOWER tip with the scoop down) until within FAR_TURN_TOLERANCE_DEGREES of 90, at
    // most FAR_TURN_TIMEOUT_MS; turning on the way in it reached the FLOWER still sideways (simulated).
    static final double FAR_TURN_TOLERANCE_DEGREES = 15;
    static final long FAR_TURN_TIMEOUT_MS = 1500;
    // FLOWER approaches drive their last FLOWER_APPROACH_INCHES at no more than this speed.
    static final double FLOWER_APPROACH_INCHES = 12;
    static final double FLOWER_APPROACH_SPEED_IN_PER_S = 35;

    // A path counts as arrived once the robot is within END_STALL_DISTANCE of its end and has moved
    // less than END_STALL_MOVE and turned less than END_STALL_TURN over END_STALL_MS.
    static final double END_STALL_DISTANCE_INCHES = 1.5;
    static final double END_STALL_MOVE_INCHES = 0.2;
    static final double END_STALL_TURN_DEGREES = 1.0;
    static final long END_STALL_MS = 200;
    static final long PATH_TIMEOUT_MS = 4000;
    static final long BAD_POSE_ABORT_MS = 250;

    // 09 (up field, line-up, push): it ends, and with it the routine, when the robot reaches the end of
    // the push, or stops on the push (within PUSH_STALL_INCHES of the end: a partner that does not
    // move), or after PUSH_TIMEOUT_MS. The push line is 23 in long.
    static final double PUSH_STALL_INCHES = 25;
    static final long PUSH_TIMEOUT_MS = 4500;

    // TurretAimCommand works in the TeleOp Pinpoint frame (field center 72 in); Pedro's is 70.75 in.
    static final double PEDRO_TO_TURRET_FRAME_INCHES = 72.0 - 70.75;

    private final Red3Tips_2_0 route = new Red3Tips_2_0();
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
    private double feedSinceMs = -1;      // clock ms since the intake and feed last started; -1 = stopped
    private boolean preFeeding;           // the pre-feed of the next volley has started
    private Pose shotSpot;                // end of the drive to the next volley's spot; null at the start
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
        turretAim = new TurretAimCommand(this::turretPose, () -> false, turret);
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
                telemetry.addData(TAG, "Ready. Place the robot at (%.1f, %.1f) in, %.0f deg.",
                        route.start.x(), route.start.y(), Math.toDegrees(route.start.heading()));
                telemetry.addData("Flywheel 00 / 02 / Far flower / 06 / 08", "%.0f / %.0f / %.0f / %.0f / %.0f rpm",
                        PRELOAD_SHOT_RPM, GARDEN_SHOT_RPM, FAR_FLOWER_SHOT_RPM, PARTNER_SHOT_RPM,
                        SIDE_FLOWER_SHOT_RPM);
                telemetry.update();
                sleep(50);
            }
            if (isStopRequested()) return;

            follower.setPose(route.start);
            clock.reset();
            shooter.setTargetRpm(PRELOAD_SHOT_RPM);
            // Set, not toggled: TurretAimCommand's default is on, and a toggle would turn it off.
            turretAim.setAimEnabled(true);
            routine = autoRoutine();
            schedule(shooterCommand, turretAim, routine);
            while (opModeIsActive() && routine.isScheduled()) {
                run();
            }
            RobotLog.ii(TAG, "Routine ended at %.1f s", clock.seconds());
        } finally {
            // Also when the field stops AUTO at 30 s: the TeleOp "Red After 3Tips" starts from here.
            if (follower != null && poseValid(follower.pose())) {
                Pose pose = follower.pose();
                AutoEndPose.record(NAME, new Pose2D(DistanceUnit.INCH, pose.x() + PEDRO_TO_TURRET_FRAME_INCHES,
                        pose.y() + PEDRO_TO_TURRET_FRAME_INCHES, AngleUnit.RADIANS, pose.heading()));
                RobotLog.ii(TAG, "End pose handed to the TeleOp: Pedro (%.2f, %.2f, %.1f deg)", pose.x(), pose.y(),
                        Math.toDegrees(pose.heading()));
            }
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
                // Tip 1: the 4 preloads from the start, before driving (turret aims at (60, 60)).
                shootPreload("00 Preload / start"),

                // The RPM is set before the Garden pickup, so the intake runs on without a gap into 02.
                setRpm(GARDEN_SHOT_RPM),
                collect(pickupWithBackOff("01 Garden", route.path101Garden())),
                keepIntakeFor(PICKUP_EXTRA_INTAKE_MS, followToShot("02 Garden load / Far flower foot",
                        route.path202FarFlowerFoot(), PRE_FEED_START_INCHES, this::startPreFeed)),
                // The Garden balls go before the robot goes into the FLOWER. The scoop comes down as
                // they fire (pointing away from the FLOWER), ready for 03.
                shoot("02 Garden load / Far flower foot", this::lowerScoop),

                // Tip 2: into the Far flower empty, its 4 balls fired from inside.
                setRpm(FAR_FLOWER_SHOT_RPM),
                turnInPlace("03 Far flower / turn", farFlowerFacing(), FAR_TURN_TOLERANCE_DEGREES,
                        FAR_TURN_TIMEOUT_MS),
                collectFromFlower(followIntoFlower("03 Far flower", slowFlowerApproach(route.path303FarFlower()),
                        FLOWER_STALL_INCHES)),
                shoot("03 Far flower load / in the FLOWER"),

                // Out of the FLOWER to the partner preloads, the intake on all the way; the scoop is
                // raised on the way out. 06 fires at the end of the lane.
                setRpm(PARTNER_SHOT_RPM),
                collect(raiseScoopOnceClear(followPath("04-05 Partner preloads", route::path405PartnerPreloads,
                        new Arrival().dwell(PARTNER_DWELL_MS).stallWithin(PICKUP_STALL_INCHES)))),
                keepIntakeFor(PICKUP_EXTRA_INTAKE_MS, followToShot("06 Partner load / near",
                        route.path606PartnerShot(), PRE_FEED_START_INCHES, this::startPreFeed)),
                shoot("06 Partner load / near"),

                // Tip 3, always (no time gate). 07 turns in onto the Side flower's axis from the 06 spot
                // (the scoop comes down as it starts); 08 fires at the end of the way out.
                setRpm(SIDE_FLOWER_SHOT_RPM),
                collectFromFlower(followIntoFlower("07 Side flower",
                        Paths.path(route.path706TurnIn(), slowFlowerApproach(route.path707SideFlower())),
                        SIDE_FLOWER_STALL_INCHES)),
                followToShot("08 Flower load / near", route.path808SideFlowerShot(), Double.POSITIVE_INFINITY,
                        this::startPreFeed),
                shoot("08 Flower load / near", this::raiseScoop),

                new InstantCommand(this::stopScoring),
                followPath("09 Park / push partner", route::path909PushPartner,
                        new Arrival().stallWithin(PUSH_STALL_INCHES).timeout(PUSH_TIMEOUT_MS)));
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
                RobotLog.ww(TAG, "Stopped on %s: Pinpoint pose invalid", step);
                cancelCommands();
            }
        } else {
            badPoseSinceMs = -1;
        }
        super.run();
        showTelemetry();
    }

    // ---------------------------------------------------------------- building blocks

    private Command setRpm(double rpm) {
        return new InstantCommand(() -> shooter.setTargetRpm(rpm));
    }

    /** {@code path} with its last FLOWER_APPROACH_INCHES slowed down (the two-tip's split). */
    private static Path slowFlowerApproach(Path path) {
        return Red2TipsAuto.slowFinish(path, FLOWER_APPROACH_INCHES, FLOWER_APPROACH_SPEED_IN_PER_S);
    }

    /**
     * Runs the intake (gate closed, at the pickup height IntakeConstants.LIFT_POSITION) until
     * {@code drive} finishes, then lowers the intake to TRAVEL_LIFT_POSITION: the pickup is over and
     * the next step drives off in the same loop. The motors stay on; the next step decides how long.
     */
    private Command collect(Command drive) {
        return new ParallelDeadlineGroup(latch(drive), new StartEndCommand(
                () -> {
                    intake.setLiftPosition(IntakeConstants.LIFT_POSITION);
                    runIntake();
                },
                () -> {
                    intake.setLiftPosition(TRAVEL_LIFT_POSITION);
                    RobotLog.ii(TAG, "%s: pickup done, intake lowered to %.2f at %.1f s",
                            step, TRAVEL_LIFT_POSITION, clock.seconds());
                },
                intake, gate));
    }

    /**
     * {@link #collect} at a FLOWER with the flower scoop lowered. The scoop stays down when the pickup
     * ends; it is raised as the next volley fires, and whenever the routine parks or stops.
     */
    private Command collectFromFlower(Command drive) {
        return new SequentialCommandGroup(new InstantCommand(flowerScoop::lower, flowerScoop), collect(drive));
    }

    /** 02's end facing the Far flower: where 03 turns on the spot before going in. */
    private Pose farFlowerFacing() {
        Path in = route.path303FarFlower();
        return new Pose(in.get(0).x(), in.get(0).y(), in.endPose().heading());
    }

    /**
     * Holds {@code target} (Pedro's hold: position and heading) and finishes once the heading is within
     * {@code toleranceDeg} of the target's, or after {@code timeoutMs}. The next step's follow() takes
     * over the rest of the turn.
     */
    private Command turnInPlace(String name, Pose target, double toleranceDeg, long timeoutMs) {
        return new CommandBase() {
            private final ElapsedTime timer = new ElapsedTime();
            private boolean done;

            @Override
            public void initialize() {
                timer.reset();
                done = false;
                step = name;
                RobotLog.ii(TAG, "%s: start at %.1f s", name, clock.seconds());
                follower.hold(target);
            }

            @Override
            public void execute() {
                Pose pose = follower.pose();
                double off = poseValid(pose) ? Math.abs(headingDifferenceDegrees(pose, target)) : Double.POSITIVE_INFINITY;
                if (off <= toleranceDeg) {
                    RobotLog.ii(TAG, "%s: done at %.1f s, %.1f deg off, robot %s", name, clock.seconds(), off,
                            describe(pose, target));
                    done = true;
                } else if (timer.milliseconds() >= timeoutMs) {
                    lastTimeout = name;
                    RobotLog.ww(TAG, "%s: timed out after %.0f ms, %.1f deg off", name, timer.milliseconds(), off);
                    done = true;
                }
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
     * Runs {@code drive} (out of the Far flower) and raises the flower scoop once the robot is
     * FAR_SCOOP_CLEAR_INCHES from where the step started, or when {@code drive} ends if that is first.
     */
    private Command raiseScoopOnceClear(Command drive) {
        return new ParallelDeadlineGroup(latch(drive), new CommandBase() {
            private Pose from;

            @Override
            public void initialize() {
                from = follower.pose();
            }

            @Override
            public boolean isFinished() {
                Pose pose = follower.pose();
                return !poseValid(from) || (poseValid(pose) && distance(pose, from) >= FAR_SCOOP_CLEAR_INCHES);
            }

            @Override
            public void end(boolean interrupted) {
                flowerScoop.raise();
                Pose pose = follower.pose();
                RobotLog.ii(TAG, "%s: flower scoop raised %.1f in from the FLOWER at %.1f s", step,
                        poseValid(from) && poseValid(pose) ? distance(pose, from) : Double.NaN, clock.seconds());
            }
        });
    }

    /** Raises the flower scoop as a volley fires (passed to {@link #shoot(String, Runnable)}). */
    private void raiseScoop() {
        flowerScoop.raise();
        RobotLog.ii(TAG, "%s: flower scoop raised as the volley fires at %.1f s", step, clock.seconds());
    }

    /** Lowers the flower scoop as a volley fires (passed to {@link #shoot(String, Runnable)}). */
    private void lowerScoop() {
        flowerScoop.lower();
        RobotLog.ii(TAG, "%s: flower scoop lowered as the volley fires at %.1f s", step, clock.seconds());
    }

    /**
     * Runs {@code drive} (the path away from a pickup) with the intake and feed still on (gate closed)
     * for its first {@code ms}, then stops them, unless the volley's pre-feed has started by then.
     */
    private Command keepIntakeFor(long ms, Command drive) {
        return new ParallelDeadlineGroup(latch(drive), new CommandBase() {
            private final ElapsedTime timer = new ElapsedTime();

            @Override
            public void initialize() {
                timer.reset();
                runIntake();
            }

            @Override
            public boolean isFinished() {
                return timer.milliseconds() >= ms;
            }

            @Override
            public void end(boolean interrupted) {
                if (!preFeeding) stopIntake();
                RobotLog.ii(TAG, "%s: intake kept on %.0f ms after the pickup", step, timer.milliseconds());
            }
        });
    }

    /**
     * Runs {@code command} and stays finished once it has finished. Every deadline goes through this:
     * FTCLib 2.1.1's ParallelDeadlineGroup ends its deadline and only then asks deadline.isFinished(),
     * and a SequentialCommandGroup answers false after end() (its index is reset to -1), so a group
     * deadline such as {@link #pickupWithBackOff} never let the pickup end. (A ParallelRaceGroup
     * instead ends it twice and throws IndexOutOfBoundsException.)
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

    /** When a path step counts as arrived and how long it then stays; see {@link #followPath}. */
    private static final class Arrival {
        long dwellMs;                                         // stay this long after arriving
        double withinInches;                                  // arrived this close to the end (0 = off)
        double stallWithinInches = END_STALL_DISTANCE_INCHES; // a stop this close to the end is arriving
        double hookWithinInches;                              // run hook once, this close to the end
        Runnable hook;                                        // or null
        long timeoutMs = PATH_TIMEOUT_MS;                     // move on after this long, whatever happens
        boolean endsAtShotSpot;                               // the end is the next volley's spot

        Arrival dwell(long ms) {
            dwellMs = ms;
            return this;
        }

        Arrival within(double inches) {
            withinInches = inches;
            return this;
        }

        Arrival stallWithin(double inches) {
            stallWithinInches = inches;
            return this;
        }

        Arrival when(double withinInches, Runnable action) {
            hookWithinInches = withinInches;
            hook = action;
            return this;
        }

        Arrival timeout(long ms) {
            timeoutMs = ms;
            return this;
        }

        Arrival endsAtShotSpot() {
            endsAtShotSpot = true;
            return this;
        }
    }

    /**
     * Follows {@code path} to a shot spot and runs {@code startPreFeed}, which starts the pre-volley
     * feed (see PRE_FEED_MS), once the robot is within {@code preFeedWithinInches} of the spot
     * (infinity: from the start of the path).
     */
    private Command followToShot(String name, Path path, double preFeedWithinInches, Runnable startPreFeed) {
        return followPath(name, () -> path, new Arrival().when(preFeedWithinInches, startPreFeed).endsAtShotSpot());
    }

    /**
     * Follows {@code path} into a FLOWER and moves on FLOWER_MAX_MS after reaching it, whether or not
     * Pedro has finished, and in any case FLOWER_STEP_MAX_MS after starting.
     */
    private Command followIntoFlower(String name, Path path, double stallWithinInches) {
        return followPath(name, () -> path, new Arrival().dwell(FLOWER_MAX_MS).within(FLOWER_CONTACT_INCHES)
                .stallWithin(stallWithinInches).timeout(FLOWER_STEP_MAX_MS));
    }

    /**
     * Follows the path from {@code path} (built when the step starts) and finishes {@code how.dwellMs}
     * after arriving. Arriving is the first of: Pedro finished the path; the robot within
     * {@code how.withinInches} of the end; the robot stopped (see END_STALL) within
     * {@code how.stallWithinInches} of the end. After {@code how.timeoutMs} the step stops the drive
     * and finishes at once. Otherwise Pedro keeps pushing toward the end until the next step's
     * follow() replaces the path.
     */
    private Command followPath(String name, Supplier<Path> path, Arrival how) {
        return new CommandBase() {
            private final ElapsedTime timer = new ElapsedTime();
            private final StallCheck stall = new StallCheck();
            private Pose end;
            private double arrivedMs;
            private boolean hookRun;
            private boolean done;

            @Override
            public void initialize() {
                timer.reset();
                arrivedMs = -1;
                hookRun = false;
                done = false;
                stall.reset();
                step = name;
                RobotLog.ii(TAG, "%s: start at %.1f s", name, clock.seconds());
                Path current = path.get();
                end = current.endPose();
                if (how.endsAtShotSpot) shotSpot = end;
                follower.follow(current);
            }

            @Override
            public void execute() {
                double now = timer.milliseconds();
                if (arrivedMs < 0) {
                    Pose pose = follower.pose();
                    double toEnd = poseValid(pose) ? distance(pose, end) : Double.POSITIVE_INFINITY;
                    if (how.hook != null && !hookRun && toEnd <= how.hookWithinInches) {
                        hookRun = true;
                        how.hook.run();
                    }
                    boolean busy = follower.isBusy();
                    boolean close = busy && toEnd <= how.withinInches;
                    boolean stalled = busy && toEnd <= how.stallWithinInches && stall.stopped(now, pose);
                    if (toEnd > how.stallWithinInches) stall.reset();
                    if (busy && !close && !stalled) {
                        if (now < how.timeoutMs) return;
                        stopDrive();
                        lastTimeout = name;
                        RobotLog.ww(TAG, "%s: timed out after %.0f ms, %.1f in from the end", name, now, toEnd);
                        done = true;
                        return;
                    }
                    if (close) {
                        RobotLog.ii(TAG, "%s: reached at %.1f s, %.1f in from the end; leaving in %d ms",
                                name, clock.seconds(), toEnd, how.dwellMs);
                    } else if (stalled) {
                        RobotLog.ii(TAG, "%s: stopped %.2f in short of the end, counted as arrived", name, toEnd);
                    } else {
                        RobotLog.ii(TAG, "%s: Pedro done at %.1f s, robot %s", name, clock.seconds(), describe(pose, end));
                    }
                    // A stall counts from when the robot stopped, not from when it was detected.
                    arrivedMs = stalled && !close ? stall.sinceMs() : now;
                }
                done = now - arrivedMs >= how.dwellMs;
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

    /** True once the pose has moved less than END_STALL_MOVE and turned less than END_STALL_TURN for a window. */
    private static final class StallCheck {
        private final long windowMs;
        private Pose from;
        private double fromMs;

        StallCheck() {
            this(END_STALL_MS);
        }

        StallCheck(long windowMs) {
            this.windowMs = windowMs;
        }

        void reset() {
            from = null;
        }

        boolean stopped(double nowMs, Pose pose) {
            if (!poseValid(pose)) {
                from = null;
                return false;
            }
            if (from == null || distance(pose, from) > END_STALL_MOVE_INCHES
                    || Math.abs(headingDifferenceDegrees(pose, from)) > END_STALL_TURN_DEGREES) {
                from = pose;
                fromMs = nowMs;
                return false;
            }
            return nowMs - fromMs >= windowMs;
        }

        double sinceMs() {
            return fromMs;
        }
    }

    /**
     * Garden pickup: drives {@code path}, collects PICKUP_DWELL_MS there, backs
     * straight off PICKUP_BACK_OFF_INCHES (5 cm, away from the intake side), drives back to the spot it
     * stopped at and collects PICKUP_DWELL_MS again. Wrap it in collect() so the intake runs throughout.
     */
    private Command pickupWithBackOff(String name, Path path) {
        Pose[] spot = new Pose[1];
        return new SequentialCommandGroup(
                followPath(name, () -> path, new Arrival().dwell(PICKUP_DWELL_MS).stallWithin(PICKUP_STALL_INCHES)),
                new InstantCommand(() -> {
                    Pose pose = follower.pose();
                    spot[0] = poseValid(pose) ? pose : path.endPose();
                }),
                followPath(name + " / back off", () -> Paths.line(spot[0], backOff(spot[0])).constant(spot[0]),
                        new Arrival().within(PICKUP_NEAR_INCHES)),
                followPath(name + " / return", () -> {
                    Pose here = follower.pose();
                    // A zero-length line is not followable; if the robot did not move, return from the
                    // planned back-off point instead.
                    Pose from = poseValid(here) && distance(here, spot[0]) > 0.25 ? here : backOff(spot[0]);
                    return Paths.line(from, spot[0]).constant(spot[0]);
                }, new Arrival().dwell(PICKUP_DWELL_MS).within(PICKUP_NEAR_INCHES)));
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

    /** Runs the intake and feed motors with the gate closed; keeps the time since they last started. */
    private void runIntake() {
        gate.setOpen(false);
        intake.setPowers(IntakeConstants.INTAKE_MOTOR_POWER, IntakeConstants.FEED_MOTOR_POWER);
        if (feedSinceMs < 0) feedSinceMs = clock.milliseconds();
    }

    private void stopIntake() {
        intake.stop();
        feedSinceMs = -1;
    }

    /**
     * Starts the pre-volley feed, once per volley (see PRE_FEED_MS): from here on only the volley stops
     * the motors. Called by the approach to a shot spot or, at the latest, when the volley starts.
     */
    private void startPreFeed() {
        if (preFeeding) return;
        preFeeding = true;
        runIntake();
    }

    /**
     * Waits until the intake and feed have run PRE_FEED_MS, the flywheel and turret are ready and the
     * robot is at the shot spot (SHOT_SPOT_INCHES, SHOT_STILL_MS), opens the gate (motors paused for the
     * TeleOp pre-open time), then feeds for one volley.
     */
    private final class ShootCommand extends CommandBase {
        private final String name;
        private final long readyTimeoutMs;
        private final Runnable onFire;     // run when the gate opens, or null
        private final ElapsedTime timer = new ElapsedTime();
        private final StallCheck still = new StallCheck(SHOT_STILL_MS);
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
            still.reset();
            step = name + " (shoot)";
            RobotLog.ii(TAG, "%s: shoot at %.1f s", name, clock.seconds());
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
                boolean preFed = clock.milliseconds() - feedSinceMs >= PRE_FEED_MS;
                Pose pose = follower.pose();
                boolean robotStill = still.stopped(now, pose);
                double off = shotSpot == null ? 0
                        : poseValid(pose) ? distance(pose, shotSpot) : Double.POSITIVE_INFINITY;
                boolean atSpot = shotSpot == null || (off <= SHOT_SPOT_INCHES && robotStill);
                shotStatus = String.format("waiting: flywheel %s, turret %s, pre-feed %s, robot %s",
                        rpmStable ? "ok" : "spinning", turretSettled ? "ok" : "moving", preFed ? "ok" : "running",
                        atSpot ? "ok" : String.format("%.1f in off%s", off, robotStill ? "" : ", moving"));
                boolean ready = rpmStable && turretSettled && atSpot;
                if (preFed && (ready || now >= readyTimeoutMs)) {
                    if (!ready) {
                        RobotLog.ww(TAG, "%s: fired on timeout (%s)", name, shotStatus);
                    }
                    RobotLog.ii(TAG, "%s: fired %.0f ms into the step, %.0f cm from the target, robot %s", name,
                            now, turretAim.getTargetDistanceCm(), shotSpot == null
                                    ? (poseValid(pose) ? String.format("at (%.2f, %.2f, %.1f deg)", pose.x(),
                                            pose.y(), Math.toDegrees(pose.heading())) : "pose invalid")
                                    : describe(pose, shotSpot));
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
            stopIntake();
            gate.setOpen(false);
            preFeeding = false;                   // the next volley runs its own pre-feed
            shotSpot = null;
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
        if (flowerScoop != null) flowerScoop.raise();   // a FLOWER pickup leaves it down until the next volley
    }

    /** Before parking: flywheel off, turret centered, intake off, gate closed, scoop raised. */
    private void stopScoring() {
        turretAim.setAimEnabled(false);                       // disabled aim re-centers the turret
        shooterCommand.cancel();                              // ShooterCommand.end() stops the flywheel
        stopIntake();
        preFeeding = false;
        gate.setOpen(false);
        flowerScoop.raise();                                  // down if the 08 volley did not fire
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

    /** "x in and y deg from (target), at (pose)" for the log. */
    private static String describe(Pose pose, Pose target) {
        if (!poseValid(pose)) return "pose invalid";
        return String.format("%.2f in and %.1f deg from (%.1f, %.1f), at (%.2f, %.2f, %.1f deg)",
                distance(pose, target), headingDifferenceDegrees(pose, target), target.x(), target.y(),
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }

    /** Heading of {@code a} minus heading of {@code b}, in degrees within +/-180. */
    private static double headingDifferenceDegrees(Pose a, Pose b) {
        return Math.toDegrees(Math.IEEEremainder(a.heading() - b.heading(), 2 * Math.PI));
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
        if (lastTimeout != null) telemetry.addData("Last timeout", lastTimeout);
        telemetry.addData("Pose (in, deg)", "%.1f, %.1f, %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Shot", shotStatus);
        telemetry.addData("Flower scoop", "%s (%.2f)",
                flowerScoop.isRaised() ? "raised" : "lowered", flowerScoop.getPosition());
        shooter.addTelemetry(telemetry);
        turretAim.addTelemetry(telemetry);
        telemetry.update();
    }
}
