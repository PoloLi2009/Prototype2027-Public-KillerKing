package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
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
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@Autonomous(name = Red3Tips3Auto.NAME, group = "Auto")
public final class Red3Tips3Auto extends CommandOpMode {
    public static final String NAME = "Red3Tips3.0";
    private final Red3Tips_3_0 route = new Red3Tips_3_0();
    private final ElapsedTime clock = new ElapsedTime();
    private Follower follower;
    private Intake intake;
    private Gate gate;
    private FlowerScoop flowerScoop;
    private Shooter shooter;
    private Turret turret;
    private TurretAimCommand turretAim;
    private ShooterCommand shooterCommand;
    private String lastTimeout;
    private String step;

    @Override
    public void initialize() {
        Constants.foresightConfig.maxVelocityConstraint.set(ForesightConfig.Constraint.NONE);
        follower = Constants.create(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        flowerScoop = new FlowerScoop(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        turretAim = new TurretAimCommand(() -> fieldPose(follower.pose()), () -> false, turret);
        shooterCommand = new ShooterCommand(shooter, this::getRuntime, turretAim::getTargetDistanceCm);
        follower.setPose(route.start);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        reset();
        Command routine = null;
        try {
            if (Constants.foresightConfig == null) {
                telemetry.addLine("Pedro Foresight calibration required.");
                telemetry.update();
                waitForStart();
                return;
            }
            initialize();
            telemetry.addLine("Red 3.0 | (54.37, 9.44), intake facing 270 degrees");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;
            clock.reset();
            routine = autoRoutine();
            schedule(shooterCommand, turretAim, routine);
            while (opModeIsActive() && routine.isScheduled()) {
                follower.update();
                run();
                telemetry.addData("3.0", "%s | %.2f s", step, clock.seconds());
                if (lastTimeout != null) telemetry.addData("Stopped", lastTimeout);
                telemetry.update();
            }
        } finally {
            if (routine != null) routine.cancel();
            if (shooterCommand != null) shooterCommand.cancel();
            if (turretAim != null) turretAim.cancel();
            if (intake != null) intake.stop();
            if (gate != null) gate.setOpen(false);
            if (shooter != null) shooter.stop();
            if (follower != null) {
                Pose2D field = fieldPose(follower.pose());
                if (routine != null && field != null) AutoEndPose.record(NAME, field);
                follower.stop();
                follower.drivetrain.stop();
            }
            reset();
        }
    }

    private static boolean validPose(Pose p) {
        return p != null && Double.isFinite(p.x()) && Double.isFinite(p.y()) && Double.isFinite(p.heading());
    }

    private static Pose2D fieldPose(Pose p) {
        if (!validPose(p)) return null;
        return new Pose2D(DistanceUnit.INCH, p.x() + 1.25, p.y() + 1.25, AngleUnit.RADIANS, p.heading());
    }

    private static double distance(Pose a, Pose b) {
        return Math.hypot(a.x() - b.x(), a.y() - b.y());
    }

    private static double angleError(Pose a, Pose b) {
        return Math.abs(Math.toDegrees(Math.IEEEremainder(a.heading() - b.heading(), 2 * Math.PI)));
    }

    private enum Stage {
        PRELOAD, GARDEN, BACK_OFF, RETURN, GARDEN_SHOT, FAR_FLOWER, FAR_SHOT,
        PARTNER, PARTNER_SHOT, SIDE_FLOWER, SIDE_SHOT, LINE_UP, PUSH, DONE, FAILED
    }

    private Command autoRoutine() {
        return new Routine();
    }

    private final class Routine extends CommandBase {
        private Stage stage;
        private Pose goal, pickupSpot, flowerContact, stillFrom;
        private double entered, stillSince, rpmSince, contactSince, prefeedSince;
        private double gateOpened, feedStarted, invalidSince;
        private boolean pickupHeight, retreatTried;
        private Pose exitOrigin, retreatTarget;
        private double retreatStart;
        private int volleys;

        @Override
        public void initialize() {
            prefeedSince = -1;
            invalidSince = -1;
            lastTimeout = null;
            turretAim.setAimEnabled(true);
            begin(Stage.PRELOAD);
        }

        private void begin(Stage next) {
            stage = next;
            step = next.name();
            entered = clock.seconds();
            stillFrom = null;
            exitOrigin = retreatTarget = null;
            retreatTried = false;
            rpmSince = contactSince = gateOpened = feedStarted = -1;
            RobotLog.ii(NAME, "%.2f s %s", entered, step);
            Pose p = follower.pose();
            switch (next) {
                case PRELOAD:
                    travelHeight();
                    shooter.setTargetRpm(Red2TipsAuto.PRELOAD_SHOT_RPM);
                    goal = route.start;
                    prefeed();
                    break;
                case GARDEN:
                    travelHeight();
                    shooter.setTargetRpm(Red2TipsAuto.GARDEN_SHOT_RPM);
                    follow(route.gardenPickup());
                    break;
                case BACK_OFF:
                    Pose back = new Pose(p.x() - Math.cos(p.heading()) * 5 / 2.54,
                            p.y() - Math.sin(p.heading()) * 5 / 2.54, p.heading());
                    follow(Paths.line(p, back).constant(back));
                    break;
                case RETURN:
                    follow(Paths.line(p, pickupSpot).constant(pickupSpot));
                    break;
                case GARDEN_SHOT:
                    travelHeight();
                    follow(route.gardenToShot(p));
                    break;
                case FAR_FLOWER:
                    shooter.setTargetRpm(Red2TipsAuto.FAR_FLOWER_SHOT_RPM);
                    flowerScoop.lower();
                    follow(route.farPickup(p));
                    break;
                case FAR_SHOT:
                    travelHeight();
                    exitOrigin = p;
                    follow(route.farExit(p));
                    break;
                case PARTNER:
                    flowerContact = null;
                    travelHeight();
                    shooter.setTargetRpm(Red2TipsAuto.NEAR_FLOWER_SHOT_RPM);
                    follow(route.partnerPickup(p));
                    break;
                case PARTNER_SHOT:
                    travelHeight();
                    exitOrigin = p;
                    follow(route.partnerExit(p));
                    break;
                case SIDE_FLOWER:
                    flowerScoop.lower();
                    follow(route.sidePickup(p));
                    break;
                case SIDE_SHOT:
                    travelHeight();
                    exitOrigin = p;
                    follow(route.sideExit(p));
                    break;
                case LINE_UP:
                    stopFeed();
                    shooterCommand.cancel();
                    shooter.stop();
                    turretAim.setAimEnabled(false);
                    follow(route.lineUp(p));
                    break;
                case PUSH:
                    follow(route.push());
                    break;
                case DONE:
                case FAILED:
                    follower.stop();
                    follower.drivetrain.stop();
                    break;
            }
        }

        private void follow(Path path) {
            goal = path.endPose();
            follower.follow(path);
        }

        private void travelHeight() {
            intake.setLiftPosition(0.50);
            pickupHeight = false;
        }

        private void prefeed() {
            if (prefeedSince >= 0) return;
            gate.setOpen(false);
            intake.setPowers(IntakeConstants.INTAKE_MOTOR_POWER, IntakeConstants.FEED_MOTOR_POWER);
            prefeedSince = clock.seconds();
        }

        private void stopFeed() {
            intake.stop();
            gate.setOpen(false);
            prefeedSince = -1;
        }

        @Override
        public void execute() {
            double now = clock.seconds();
            if (now >= 29.7) {
                fail("29.7 s deadline");
                return;
            }
            Pose p = follower.pose();
            if (!validPose(p)) {
                follower.drivetrain.stop();
                stillFrom = null;
                rpmSince = contactSince = -1;
                if (invalidSince < 0) invalidSince = now;
                if (gateOpened >= 0 || now - invalidSince >= 0.25) fail("lost pose");
                return;
            }
            invalidSince = -1;
            if (stillFrom == null || distance(p, stillFrom) > 0.2 || angleError(p, stillFrom) > 1) {
                stillFrom = p;
                stillSince = now;
            }
            double off = distance(p, goal);
            switch (stage) {
                case PRELOAD:
                case GARDEN_SHOT:
                case FAR_SHOT:
                case PARTNER_SHOT:
                case SIDE_SHOT:
                    shoot(now, p, off);
                    break;
                case GARDEN:
                case RETURN:
                case PARTNER:
                case FAR_FLOWER:
                case SIDE_FLOWER:
                    collect(now, p, off);
                    break;
                case BACK_OFF:
                    if (off <= 0.30) begin(Stage.RETURN);
                    else if (now - entered > 0.85) fail("Garden back-off blocked");
                    break;
                case LINE_UP:
                    if (off <= 0.75 && angleError(p, goal) <= 3 && now - stillSince >= 0.06) begin(Stage.PUSH);
                    else if (now - entered > 3.8) fail("rear face did not align");
                    break;
                case PUSH:
                    if (Math.abs(p.x() - route.square.x()) > 1.5 || angleError(p, goal) > 6) {
                        fail("push lost alignment");
                    } else if (off <= 0.65 && angleError(p, goal) <= 3) {
                        RobotLog.ii(NAME, "Park endpoint verified, %d commanded volleys at %.2f s", volleys, now);
                        begin(Stage.DONE);
                    } else if (now - entered > 2.8) fail("push blocked");
                    break;
                default: break;
            }
        }

        private void collect(double now, Pose p, double off) {
            boolean flower = stage == Stage.FAR_FLOWER || stage == Stage.SIDE_FLOWER;
            if (!pickupHeight && off <= (flower ? 18 : 13)) {
                intake.setLiftPosition(IntakeConstants.LIFT_POSITION);
                pickupHeight = true;
                prefeed();
            }
            boolean atPickup = off <= (flower ? 5.0 : 0.35);
            if (now - stillSince >= 0.17 && angleError(p, goal) <= 8) {
                if (stage == Stage.FAR_FLOWER) atPickup |= Math.abs(p.x() - goal.x()) < 2 && p.y() >= 123.0 && off <= 13;
                else if (stage == Stage.SIDE_FLOWER) atPickup |= Math.abs(p.y() - goal.y()) < 2 && p.x() <= 18 && off <= 10;
                else atPickup |= off <= (stage == Stage.RETURN ? 0.35 : 2.5);
            }
            if (!atPickup) contactSince = -1;
            else if (contactSince < 0) contactSince = now;
            double dwell = flower ? 0.65 : stage == Stage.PARTNER ? 0.45 : 0.35;
            if (contactSince >= 0 && now - contactSince >= dwell) {
                if (flower) flowerContact = p;
                if (stage == Stage.GARDEN) pickupSpot = p;
                begin(Stage.values()[stage.ordinal() + 1]);
            } else if (now - entered > (flower ? 2.6 : stage == Stage.RETURN ? 1.25 : 4.0)) {
                fail("pickup timeout");
            }
        }

        private void shoot(double now, Pose p, double off) {
            if (gateOpened >= 0) {
                if (feedStarted < 0 && now - gateOpened >= GateConstants.PRE_OPEN_SECONDS) {
                    intake.setPowers(IntakeConstants.INTAKE_MOTOR_POWER, IntakeConstants.AUTO_FIRE_FEED_MOTOR_POWER);
                    feedStarted = now;
                }
                if (feedStarted >= 0 && now - feedStarted >= Red3Tips2Auto.VOLLEY_MS / 1000.0) {
                    stopFeed();
                    volleys++;
                    begin(Stage.values()[stage.ordinal() + 1]);
                }
                return;
            }
            if (handleBlockedExit(now, p)) return;
            if (off < 40) prefeed();
            boolean rpmReady = Math.abs(shooter.getTargetRpm() - shooter.getFilteredRpm()) <= Red2TipsAuto.RPM_TOLERANCE;
            if (!rpmReady) rpmSince = -1;
            else if (rpmSince < 0) rpmSince = now;
            boolean aimed = !turretAim.isInDeadZone() && Double.isFinite(turretAim.getTargetDistanceCm())
                    && Math.abs(turret.getEstimatedOffsetDegrees(System.nanoTime() / 1e9) - turret.getOffsetDegrees()) <= 2;
            boolean clear = flowerContact == null || distance(p, flowerContact) >= 10;
            boolean ready = off <= 1.25 && angleError(p, goal) <= 5 && now - stillSince >= 0.10
                    && rpmSince >= 0 && now - rpmSince >= 0.10 && aimed && clear;
            if (ready && prefeedSince >= 0 && now - prefeedSince >= 1) {
                RobotLog.ii(NAME, "%s gate open at %.2f s, %.2f in off, %.1f cm, %.0f RPM", step, now, off,
                        turretAim.getTargetDistanceCm(), shooter.getFilteredRpm());
                intake.stop();
                gate.setOpen(true);
                if (flowerContact != null) {
                    flowerScoop.raise();
                    flowerContact = null;
                }
                gateOpened = now;
            } else if (now - entered > (stage == Stage.PRELOAD ? 3.0 : 4.8)) {
                fail(String.format("shot not ready: error %.1f in, heading error %.1f deg, RPM %.0f", off,
                        angleError(p, goal), shooter.getFilteredRpm()));
            }
        }

        // A scoop or intake pinned against the field element may prevent the curved departure.
        // Make one straight retreat along the intake axis, then rejoin from the measured pose.
        private boolean handleBlockedExit(double now, Pose p) {
            if (exitOrigin == null) return false;
            if (retreatTarget != null) {
                if (distance(p, retreatTarget) <= 0.5) {
                    retreatTarget = null;
                    if (stage == Stage.FAR_SHOT) follow(route.farExit(p));
                    else if (stage == Stage.SIDE_SHOT) follow(route.sideExit(p));
                    else follow(route.partnerExit(p));
                } else if (now - retreatStart >= 0.9) {
                    fail("straight retreat blocked");
                }
                return true;
            }
            if (!retreatTried && now - entered > 0.42 && now - stillSince > 0.24 && distance(p, exitOrigin) < 2) {
                retreatTried = true;
                retreatStart = now;
                retreatTarget = new Pose(p.x() - 4 * Math.cos(p.heading()), p.y() - 4 * Math.sin(p.heading()), p.heading());
                // Keep the shooting goal unchanged while temporarily following the retreat.
                follower.follow(Paths.line(p, retreatTarget).constant(retreatTarget));
                RobotLog.ww(NAME, "%s blocked on exit; backing off 4 in at %.2f s", step, now);
                return true;
            }
            return false;
        }

        private void fail(String reason) {
            lastTimeout = step + " / " + reason;
            RobotLog.ww(NAME, "%.2f s %s, %d completed volleys", clock.seconds(), lastTimeout, volleys);
            stopFeed();
            begin(Stage.FAILED);
        }

        @Override
        public boolean isFinished() {
            return stage == Stage.DONE || stage == Stage.FAILED;
        }

        @Override
        public void end(boolean interrupted) {
            stopFeed();
            travelHeight();
            // If stopped against a FLOWER, leave the stationary scoop where it is.
            // A normal volley raised it after the chassis cleared the fixture.
            shooter.stop();
            follower.stop();
            follower.drivetrain.stop();
        }
    }
}
