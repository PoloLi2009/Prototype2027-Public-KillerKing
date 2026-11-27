package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Pedro route for the STABLE Blue3Tips1.0: {@link Red3Tips_1_0} mirrored to the blue side.
 * Every pose is the red one rotated 180 deg about the field center (point symmetry, Pedro field
 * 141.5 in): x' = 141.5 - x, y' = 141.5 - y, heading + 180 deg (control points keep their exported 0;
 * their heading is unused). The path methods are the red ones unchanged. Generated from the red
 * route, not a Visualizer export: change the red route and regenerate, so both stay mirrored.
 */
public final class Blue3Tips_1_0 {
    public static final double FIELD_SIZE_IN = 141.5;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(87.1344, 132.061, 90);
    private final Pose point101Garden = poseFactory.of(134.5368, 131.5346, 90);
    private final Pose point101GardenControl1 = poseFactory.of(103.7999, 129.1333, 0);
    private final Pose point101GardenControl2 = poseFactory.of(133.8527, 114.9559, 0);
    private final Pose point202Gardenloadfar = poseFactory.of(98.8599, 28.2153, 90);
    private final Pose point202GardenloadfarControl1 = poseFactory.of(104.6032, 134.2956, 0);
    private final Pose point202GardenloadfarControl2 = poseFactory.of(108.2236, 107.5411, 0);
    private final Pose point202GardenloadfarControl3 = poseFactory.of(114.589, 29.5729, 0);
    private final Pose point303FarflowerStart = poseFactory.of(98.8599, 28.2153, 250);
    private final Pose point303Farflower = poseFactory.of(94.9683, 7.0074, 270);
    private final Pose point303FarflowerControl1 = poseFactory.of(95.0063, 17.5, 0);
    private final Pose point404Flowerloadfar = poseFactory.of(103.6045, 25.4831, 270);
    private final Pose point404FlowerloadfarControl1 = poseFactory.of(95.0063, 16.5, 0);
    private final Pose point505PartnerpreloadsStart = poseFactory.of(103.6045, 25.4831, 270);
    private final Pose point505Partnerpreloads = poseFactory.of(133.7612, 38.8248, 90);
    private final Pose point505PartnerpreloadsControl1 = poseFactory.of(103.8446, 15.34, 0);
    private final Pose point505PartnerpreloadsControl2 = poseFactory.of(120.6696, 6.9525, 0);
    private final Pose point606PartneravoidnearStart = poseFactory.of(133.7612, 38.8248, 90);
    private final Pose point606Partneravoidnear = poseFactory.of(106.5175, 103.5867, 0);
    private final Pose point606PartneravoidnearControl1 = poseFactory.of(117.5964, 21.0056, 0);
    private final Pose point606PartneravoidnearControl2 = poseFactory.of(108.3134, 35.4922, 0);
    private final Pose point606PartneravoidnearControl3 = poseFactory.of(118.8564, 67.952, 0);
    private final Pose point707Sideflower = poseFactory.of(132.465, 94.8817, 0);
    private final Pose point707SideflowerControl1 = poseFactory.of(117.5, 103.5, 0);
    private final Pose point707SideflowerControl2 = poseFactory.of(124.5, 94.8174, 0);
    private final Pose point808Flowerloadnear = poseFactory.of(108.2859, 106.2215, 0);
    private final Pose point808FlowerloadnearControl1 = poseFactory.of(124.5, 94.8174, 0);
    private final Pose point808FlowerloadnearControl2 = poseFactory.of(116.5, 102.5, 0);
    private final Pose point909ParkStart = poseFactory.of(108.2859, 106.2215, 0);
    private final Pose point909Park = poseFactory.of(132.1075, 41.3259, 270);
    private final Pose point909ParkControl1 = poseFactory.of(113.2332, 95.2453, 0);
    private final Pose point909ParkControl2 = poseFactory.of(133.9314, 79.3338, 0);

    public Path path101Garden() {
        return Paths.curve(start, point101GardenControl1, point101GardenControl2, point101Garden).constant(point101Garden);
    }

    public Path path202Gardenloadfar() {
        return Paths.curve(point101Garden, point202GardenloadfarControl1, point202GardenloadfarControl2, point202GardenloadfarControl3, point202Gardenloadfar).constant(point202Gardenloadfar);
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
