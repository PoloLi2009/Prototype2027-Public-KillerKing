package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Route of the EXPERIMENTAL {@link Red3TipsVisionAuto}: the first 2026-11-26 Visualizer export (02
 * shoots at 70 deg from (39.28, 109.62)), kept as its own copy so the stable {@link Red3Tips_1_0}
 * can change without moving the vision version, its recovery paths or the recovery pp.json files.
 * Same conventions: intake = robot front, Pedro field inches, headings in degrees at construction.
 *
 * <p>The front edge of the 479.5 x 348.2 mm robot touches the wall at the start, Garden, Far flower
 * and Side flower points (9.439 in from it).
 */
public final class Red3TipsVisionRoute {
    public static final double FIELD_SIZE_IN = 141.5;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(54.3656, 9.439, 270);
    private final Pose point101Garden = poseFactory.of(7.4252, 9.439, 270);
    private final Pose point101GardenControl1 = poseFactory.of(37.7001, 12.3667, 0);
    private final Pose point101GardenControl2 = poseFactory.of(7.6473, 26.5441, 0);
    private final Pose point202GardenloadfarStart = poseFactory.of(7.4252, 9.439, 270);
    private final Pose point202Gardenloadfar = poseFactory.of(39.2847, 109.6207, 70);
    private final Pose point202GardenloadfarControl1 = poseFactory.of(12, 26, 0);
    private final Pose point202GardenloadfarControl2 = poseFactory.of(20, 60, 0);
    private final Pose point202GardenloadfarControl3 = poseFactory.of(28, 92, 0);
    private final Pose point303FarflowerStart = poseFactory.of(39.2847, 109.6207, 70);
    private final Pose point303Farflower = poseFactory.of(46.4937, 132.061, 90);
    private final Pose point303FarflowerControl1 = poseFactory.of(46.4937, 124, 0);
    private final Pose point404Flowerloadfar = poseFactory.of(37.8955, 116.0169, 90);
    private final Pose point404FlowerloadfarControl1 = poseFactory.of(46.4937, 125, 0);
    private final Pose point505PartnerpreloadsStart = poseFactory.of(37.8955, 116.0169, 90);
    private final Pose point505Partnerpreloads = poseFactory.of(7.7388, 102.6752, 270);
    private final Pose point505PartnerpreloadsControl1 = poseFactory.of(37.6554, 126.16, 0);
    private final Pose point505PartnerpreloadsControl2 = poseFactory.of(20.8304, 134.5475, 0);
    private final Pose point606PartneravoidnearStart = poseFactory.of(7.7388, 102.6752, 270);
    private final Pose point606Partneravoidnear = poseFactory.of(34.9825, 37.9133, 180);
    private final Pose point606PartneravoidnearControl1 = poseFactory.of(23.9036, 120.4944, 0);
    private final Pose point606PartneravoidnearControl2 = poseFactory.of(33.1866, 106.0078, 0);
    private final Pose point606PartneravoidnearControl3 = poseFactory.of(22.6436, 73.548, 0);
    private final Pose point707Sideflower = poseFactory.of(9.439, 46.6826, 180);
    private final Pose point707SideflowerControl1 = poseFactory.of(24, 38, 0);
    private final Pose point707SideflowerControl2 = poseFactory.of(17, 46.6826, 0);
    private final Pose point808Flowerloadnear = poseFactory.of(33.2141, 35.2785, 180);
    private final Pose point808FlowerloadnearControl1 = poseFactory.of(17, 46.6826, 0);
    private final Pose point808FlowerloadnearControl2 = poseFactory.of(25, 39, 0);
    private final Pose point909ParkStart = poseFactory.of(33.2141, 35.2785, 180);
    private final Pose point909Park = poseFactory.of(9.3925, 100.1741, 90);
    private final Pose point909ParkControl1 = poseFactory.of(28.2668, 46.2547, 0);
    private final Pose point909ParkControl2 = poseFactory.of(7.5686, 62.1662, 0);

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
        return Paths.curve(point505PartnerpreloadsStart, point505PartnerpreloadsControl1, point505PartnerpreloadsControl2, point505Partnerpreloads).linear(point505PartnerpreloadsStart, point505Partnerpreloads);
    }

    public Path path606Partneravoidnear() {
        return Paths.curve(point606PartneravoidnearStart, point606PartneravoidnearControl1, point606PartneravoidnearControl2, point606PartneravoidnearControl3, point606Partneravoidnear).linear(point606PartneravoidnearStart, point606Partneravoidnear);
    }

    public Path path707Sideflower() {
        return Paths.curve(point606Partneravoidnear, point707SideflowerControl1, point707SideflowerControl2, point707Sideflower).constant(point707Sideflower);
    }

    public Path path808Flowerloadnear() {
        return Paths.curve(point707Sideflower, point808FlowerloadnearControl1, point808FlowerloadnearControl2, point808Flowerloadnear).constant(point808Flowerloadnear);
    }

    public Path path909Park() {
        return Paths.curve(point909ParkStart, point909ParkControl1, point909ParkControl2, point909Park).linear(point909ParkStart, point909Park);
    }
}
