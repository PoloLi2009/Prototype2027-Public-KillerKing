package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Route for Red3Tips2.0 (Killer King, intake = robot front, Pedro field inches, headings in degrees at
 * construction). The user's Pedro Visualizer export of 2026-11-27 (after the first robot run), with the
 * Visualizer's split pieces joined back into the auto's steps, and three changes checked by script
 * against the walls, the FLOWERS, the HIVE frame and the partner for the 479.5 x 348.2 mm robot:
 * <ul>
 * <li>04-05: the 180 deg turn (clockwise, intake toward the field) starts on 04, once the robot is
 *     11 in from where the scoop stops it at the FLOWER (the scoop is raised at 10 in), instead of at
 *     05's start: spread over 05's 24.7 in alone the robot could not keep up with it and (simulated)
 *     swung a corner into the alliance wall. Now it spreads over 32 in at no more than
 *     {@link #TURN_SPEED_IN_PER_S}; the closest point is 0.7 in from the wall, near the end of the turn
 *     (the user accepts a light touch there), for any turn rate from about 170 deg/s up (simulated;
 *     without the cap it needed 300 deg/s).
 * <li>05: the second piece's first control point continues the first piece's end tangent (the export
 *     still had the old split's control point, which turned the robot back up 1 in first).
 * <li>06: the lane's first two control points are 5 in and 2.1 in higher (y 116 and 100 instead of 111
 *     and 97.8979): the exported lane passed the partner's corner with no gap; now 2.0 in.
 * </ul>
 *
 * <p>Field model (Pedro inches, red alliance wall x = 0, audience wall y = 0):
 * <ul>
 * <li>HIVE frame, red side: x 46-48.6, y 50-91.4 (the Visualizer shape). 02 passes it 1.65 in away,
 *     the 06 lane 2.3 in.
 * <li>Red LOADING ZONE (the park zone): x 0-11, y 94.33-117.92, the fifth TILE row from the audience
 *     (manual Figure 9-2).
 * <li>Partner preloads: 4 POLLEN in a row on the zone's short edge nearest the start, y about 94.3.
 * <li>Partner robot: 16 x 16 in against the alliance wall on the start side of its preloads,
 *     x 0-16, y about 78-94.3.
 * </ul>
 *
 * <p>Shot spots (robot center to the turret target, Pedro frame (58.75, 58.75) / (58.75, 82.75)):
 * 00 start 125.7 cm; the Garden balls from {@link #farFlowerFoot} (02's end, in front of the Far
 * flower) 101.0 cm; the FLOWER's balls from inside the Far flower, where the lowered scoop stops the
 * robot 9-13 in short of {@link #farFlower}, 108-118 cm. 06 fires at the end of its lane (37.1898,
 * 31.2595), 88.8 cm from the near CELL, and 08 at the end of its way out of the Side flower (29.9919,
 * 36.4792), 92.4 cm: both in front of the CELL's opening (38 and 52 deg off its axis), as the user
 * laid them out. From the turn-in and up-field ends (19.66, 46.6183) / (17.9, 56) the balls hit the
 * CELL's side wall (2026-11-27 09:00 run).
 *
 * <p>The route ends on {@link #end}, right after the push: both robots are in the LOADING ZONE. The
 * TeleOp "Red After 3Tips" starts there (PinpointConstants.RED_AFTER_3TIPS_START_POSE, used when the
 * auto did not hand over its measured end pose). Keep the two in step.
 */
public final class Red3Tips_2_0 {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // 00 / 01: start and Garden, as in Red2Tips_1_0.
    public final Pose start = poseFactory.of(54.3656, 9.439, 270);
    private final Pose garden = poseFactory.of(6.9632, 9.9654, 270);
    private final Pose gardenControl1 = poseFactory.of(37.7001, 12.3667, 0);
    private final Pose gardenControl2 = poseFactory.of(7.6473, 26.5441, 0);

    // 02: from the Garden to the foot of the Far flower, backward at 270 deg (the export's linear
    // 270 -> 270), where the Garden balls are fired before the robot goes into the FLOWER. The exported
    // control point order is kept.
    final Pose farFlowerFoot = poseFactory.of(46.1844, 120.4723, 270);
    private final Pose farFlowerFootControl1 = poseFactory.of(36.8968, 7.2044, 0);
    private final Pose farFlowerFootControl2 = poseFactory.of(34.2938, 41.1713, 0);
    private final Pose farFlowerFootControl3 = poseFactory.of(16.8194, 115.9993, 0);
    private final Pose farFlowerFootControl4 = poseFactory.of(25.7275, 73.8934, 0);
    private final Pose farFlowerFootControl5 = poseFactory.of(48.696, 112.9044, 0);

    // 03: straight up into the Far flower at 90 deg, empty. The robot turns from 270 on the spot first;
    // with the scoop down that leaves 2.3 in to the FLOWER tip (the two-tip's 03 turned with 1.8 in).
    // The point is past where the scoop stops the robot, so it presses in.
    private final Pose farFlower = poseFactory.of(46.8236, 136.8596, 90);

    // 04 + 05: back out of the FLOWER and on to the partner preloads, turning 180 deg on the way, then
    // straight down the alliance wall onto them, intake first. The export's 04 curve (control point
    // (47.4489, 112.9232)) is split by de Casteljau at t 0.782, where the turn starts; same shape.
    private final Pose turnStart = poseFactory.of(36.5128, 119.5321, 90);
    private final Pose turnStartControl1 = poseFactory.of(47.3126, 118.1413, 0);
    private final Pose partnerApproach = poseFactory.of(29.6141, 121.8702, 48.84);
    private final Pose partnerApproachControl1 = poseFactory.of(33.5021, 119.9198, 0);
    // Speed cap on the two turning pieces (Pedro's maxVelocityConstraint, on those pieces only). 25 in/s
    // keeps the wall clear down to 150 deg/s and costs another 0.25 s.
    static final double TURN_SPEED_IN_PER_S = 30;
    private final Pose partnerTurned = poseFactory.of(8.6837, 112.7736, 270);
    private final Pose partnerTurnedControl1 = poseFactory.of(21.6056, 124.5859, 0);
    private final Pose partnerTurnedControl2 = poseFactory.of(10.0139, 118.1921, 0);
    private final Pose partnerPreloads = poseFactory.of(7.7388, 102.6752, 270);
    private final Pose partnerPreloadsControl1 = poseFactory.of(8.3261, 111.3169, 0);
    private final Pose partnerPreloadsControl2 = poseFactory.of(7.7388, 110.5992, 0);

    // 06: back off the partner and around its corner (2.0 in), down the right of the HIVE frame (2.3 in)
    // at 270 deg to the shot spot at the lane's end.
    private final Pose partnerLane = poseFactory.of(37.1898, 31.2595, 270);
    private final Pose partnerLaneControl1 = poseFactory.of(8.5, 116, 0);
    private final Pose partnerLaneControl2 = poseFactory.of(44.1147, 100, 0);
    private final Pose partnerLaneControl3 = poseFactory.of(34, 104, 0);

    // 07: turn in to 180 deg onto the Side flower's axis, then straight into it.
    private final Pose sideFlowerAxis = poseFactory.of(19.66, 46.6183, 180);
    private final Pose sideFlowerAxisControl1 = poseFactory.of(25.8715, 35.0879, 0);
    private final Pose sideFlowerAxisControl2 = poseFactory.of(25, 46.6183, 0);
    private final Pose sideFlower = poseFactory.of(9.035, 46.6183, 180);

    // 08: back out of the FLOWER along its axis and down to the shot spot, 17 in from where the robot
    // stood at the FLOWER, so the scoop is clear of it when it is raised as the volley fires.
    private final Pose sideFlowerShot = poseFactory.of(29.9919, 36.4792, 180);
    private final Pose sideFlowerShotControl1 = poseFactory.of(31.9721, 48.5125, 0);

    // 09: up field to (17.9, 56), then line up under the partner and push.
    private final Pose parkApproach = poseFactory.of(17.9, 56, 180);
    private final Pose parkApproachControl1 = poseFactory.of(22.5, 46.6183, 0);
    private final Pose parkApproachControl2 = poseFactory.of(19.5, 52.5, 0);

    // 09: turn to 270 deg and line up under the partner, back toward it; push it straight up the
    // alliance wall into the zone, back face on its side face (x 2.65-16.35 on its x 0-16). The turn is
    // over before the back meets the partner (robot center y 67.5). The push stops as soon as both
    // robots are in the zone: the robot's back at y 97.94, 3.6 in past the zone's near edge (94.33),
    // and the partner at y 97.94-113.94 (an 18 in partner still fits below 117.92). The line-up passes
    // 3.2 in from the Side flower.
    private final Pose pushStart = poseFactory.of(9.5, 65.5, 270);
    private final Pose pushStartControl1 = poseFactory.of(19.5, 61, 0);
    private final Pose pushStartControl2 = poseFactory.of(11, 58, 0);
    private static final double PUSH_SQUARE_BY = 0.5;
    // The line-up turns 90 deg within 6.3 in. 09 starts at the 08 spot, so the robot comes into it at
    // speed from the up-field piece: without a cap it fell 38 deg behind at 200 deg/s and swung a
    // corner 1.3 in into the alliance wall (simulated).
    static final double LINE_UP_SPEED_IN_PER_S = 15;
    /** Where the route ends: the end of the push, in the LOADING ZONE, the robot's back to the partner. */
    public final Pose end = poseFactory.of(9.5, 88.5, 270);

    public Path path101Garden() {
        return Paths.curve(start, gardenControl1, gardenControl2, garden).constant(garden);
    }

    public Path path202FarFlowerFoot() {
        return Paths.curve(garden, farFlowerFootControl1, farFlowerFootControl2, farFlowerFootControl3,
                farFlowerFootControl4, farFlowerFootControl5, farFlowerFoot).constant(farFlowerFoot);
    }

    public Path path303FarFlower() {
        return Paths.line(farFlowerFoot, farFlower).constant(farFlower);
    }

    public Path path405PartnerPreloads() {
        Path backOut = Paths.curve(farFlower, turnStartControl1, turnStart).constant(turnStart);
        Path turnOn04 = turnSpeed(Paths.curve(turnStart, partnerApproachControl1, partnerApproach)
                .linear(turnStart, partnerApproach));
        Path turnOn05 = turnSpeed(Paths.curve(partnerApproach, partnerTurnedControl1, partnerTurnedControl2,
                partnerTurned).linear(partnerApproach, partnerTurned));
        Path down = Paths.curve(partnerTurned, partnerPreloadsControl1, partnerPreloadsControl2, partnerPreloads)
                .constant(partnerPreloads);
        return Paths.path(backOut, turnOn04, turnOn05, down);
    }

    private static Path lineUpSpeed(Path path) {
        return Constants.foresightConfig == null ? path
                : path.with(Constants.foresightConfig.maxVelocityConstraint.at(LINE_UP_SPEED_IN_PER_S));
    }

    private static Path turnSpeed(Path path) {
        return Constants.foresightConfig == null ? path
                : path.with(Constants.foresightConfig.maxVelocityConstraint.at(TURN_SPEED_IN_PER_S));
    }

    /** 06: the lane from the partner preloads to the shot spot at its end. */
    public Path path606PartnerShot() {
        return Paths.curve(partnerPreloads, partnerLaneControl1, partnerLaneControl2, partnerLaneControl3,
                partnerLane).constant(partnerLane);
    }

    /** 07, first piece: from the 06 spot, turning to 180 deg onto the Side flower's axis. */
    public Path path706TurnIn() {
        return Paths.curve(partnerLane, sideFlowerAxisControl1, sideFlowerAxisControl2, sideFlowerAxis)
                .linear(partnerLane, sideFlowerAxis);
    }

    /** 07, second piece: straight into the Side flower along its axis. */
    public Path path707SideFlower() {
        return Paths.line(sideFlowerAxis, sideFlower).constant(sideFlower);
    }

    /** 08: out of the Side flower to the shot spot. */
    public Path path808SideFlowerShot() {
        return Paths.curve(sideFlower, sideFlowerShotControl1, sideFlowerShot).constant(sideFlowerShot);
    }

    /** 09: up field, line up under the partner, push it into the zone. */
    public Path path909PushPartner() {
        Path upField = Paths.curve(sideFlowerShot, parkApproachControl1, parkApproachControl2, parkApproach)
                .constant(parkApproach);
        Path lineUp = lineUpSpeed(Paths.curve(parkApproach, pushStartControl1, pushStartControl2, pushStart)
                .linear(parkApproach, pushStart, PUSH_SQUARE_BY));
        Path push = Paths.line(pushStart, end).constant(end);
        return Paths.path(upField, lineUp, push);
    }
}
