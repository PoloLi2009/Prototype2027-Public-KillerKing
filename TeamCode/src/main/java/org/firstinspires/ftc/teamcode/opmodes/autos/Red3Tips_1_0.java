package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Pedro Visualizer route for Red3Tips1.0 (Killer King, intake = robot front, Pedro field inches,
 * headings in degrees at construction). Same layout as the Visualizer export (class renamed because
 * {@code Paths} clashes with the Pedro import). Autonomous-only heading changes live in
 * {@link Red3TipsAuto}.
 *
 * <p>Adjusted for the measured robot size, 479.5 x 348.2 mm (18.88 x 13.71 in, center = middle of
 * that footprint): the start, Garden, Far flower and Side flower points put the front edge exactly on
 * the wall ({@link #HALF_LENGTH_IN} from it), and the Park curve's second control point keeps the
 * robot off the wall while it turns. Re-apply these if the route is re-exported.
 */
public final class Red3Tips_1_0 {
    public static final double FIELD_SIZE_IN = 141.5;
    public static final double HALF_LENGTH_IN = 479.5 / 2.0 / 25.4;   // 9.439 in

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(54.3656, HALF_LENGTH_IN, 270);
    private final Pose point101Garden = poseFactory.of(7.4252, HALF_LENGTH_IN, 270);
    private final Pose point101GardenControl1 = poseFactory.of(37.7001, 12.3667, 0);
    private final Pose point101GardenControl2 = poseFactory.of(7.6473, 26.5441, 0);
    private final Pose point202GardenloadfarStart = poseFactory.of(7.4252, HALF_LENGTH_IN, 270);
    private final Pose point202Gardenloadfar = poseFactory.of(39.2847, 109.6207, 70);
    private final Pose point202GardenloadfarControl1 = poseFactory.of(12, 26, 0);
    private final Pose point202GardenloadfarControl2 = poseFactory.of(20, 60, 0);
    private final Pose point202GardenloadfarControl3 = poseFactory.of(28, 92, 0);
    private final Pose point303FarflowerStart = poseFactory.of(39.2847, 109.6207, 70);
    private final Pose point303Farflower = poseFactory.of(46.4937, FIELD_SIZE_IN - HALF_LENGTH_IN, 90);
    private final Pose point303FarflowerControl1 = poseFactory.of(46.4937, 124, 0);
    private final Pose point404Flowerloadfar = poseFactory.of(37.8955, 116.0169, 90);
    private final Pose point404FlowerloadfarControl1 = poseFactory.of(46.4937, 125, 0);
    private final Pose point505Partnerpreloads = poseFactory.of(7.7388, 102.6752, 90);
    private final Pose point505PartnerpreloadsControl1 = poseFactory.of(25.2284, 114.0389, 0);
    private final Pose point505PartnerpreloadsControl2 = poseFactory.of(7.7905, 124.1674, 0);
    private final Pose point606Partneravoid = poseFactory.of(34.9825, 37.9133, 90);
    private final Pose point606PartneravoidControl1 = poseFactory.of(21.5181, 109.6871, 0);
    private final Pose point606PartneravoidControl2 = poseFactory.of(42.803, 100.7579, 0);
    private final Pose point707Sideflower = poseFactory.of(HALF_LENGTH_IN, 46.6826, 180);
    private final Pose point707SideflowerControl1 = poseFactory.of(24, 38, 0);
    private final Pose point707SideflowerControl2 = poseFactory.of(17, 46.6826, 0);
    private final Pose point808Flowerloadnear = poseFactory.of(33.2141, 35.2785, 180);
    private final Pose point808FlowerloadnearControl1 = poseFactory.of(17, 46.6826, 0);
    private final Pose point808FlowerloadnearControl2 = poseFactory.of(25, 39, 0);
    private final Pose point909ParkStart = poseFactory.of(33.2141, 35.2785, 180);
    private final Pose point909Park = poseFactory.of(8.2, 100.1746, 90);
    private final Pose point909ParkControl1 = poseFactory.of(28.2668, 46.2547, 0);
    private final Pose point909ParkControl2 = poseFactory.of(9.5, 64, 0);

    public Path path101Garden() {
        return Paths.curve(start, point101GardenControl1, point101GardenControl2, point101Garden).constant(point101Garden);
    }

    public Path path202Gardenloadfar() {
        return Paths.curve(point202GardenloadfarStart, point202GardenloadfarControl1, point202GardenloadfarControl2, point202GardenloadfarControl3, point202Gardenloadfar).linear(point202GardenloadfarStart, point202Gardenloadfar);
    }

    public Path path303Farflower() {
        return Paths.curve(point303FarflowerStart, point303FarflowerControl1, point303Farflower).linear(point303FarflowerStart, point303Farflower);
    }

    public Path path404Flowerloadfar() {
        return Paths.curve(point303Farflower, point404FlowerloadfarControl1, point404Flowerloadfar).constant(point404Flowerloadfar);
    }

    public Path path505Partnerpreloads() {
        return Paths.curve(point404Flowerloadfar, point505PartnerpreloadsControl1, point505PartnerpreloadsControl2, point505Partnerpreloads).constant(point505Partnerpreloads);
    }

    public Path path606Partneravoid() {
        return Paths.curve(point505Partnerpreloads, point606PartneravoidControl1, point606PartneravoidControl2, point606Partneravoid).constant(point606Partneravoid);
    }

    public Path path707Sideflower() {
        return Paths.curve(point606Partneravoid, point707SideflowerControl1, point707SideflowerControl2, point707Sideflower).constant(point707Sideflower);
    }

    public Path path808Flowerloadnear() {
        return Paths.curve(point707Sideflower, point808FlowerloadnearControl1, point808FlowerloadnearControl2, point808Flowerloadnear).constant(point808Flowerloadnear);
    }

    public Path path909Park() {
        return Paths.curve(point909ParkStart, point909ParkControl1, point909ParkControl2, point909Park).linear(point909ParkStart, point909Park);
    }
}
