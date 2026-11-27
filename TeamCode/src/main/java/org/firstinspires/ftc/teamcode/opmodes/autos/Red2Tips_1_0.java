package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Pedro Visualizer route for Red2Tips1.0 (Killer King, intake = robot front, Pedro field inches,
 * headings in degrees at construction). Poses and paths are the Visualizer export of 2026-11-27
 * unchanged, except two points. The Far flower point (end of 03, start of 04) is 5 cm (1.9685 in)
 * deeper than exported, y 134.4926 instead of 132.5241, so the intake presses further into the
 * FLOWER (the scoop was down but the balls did not come). The Far flower shot spot (end of 04, start
 * of 05) is 10 cm (3.937 in) toward the red alliance wall, x 36.9136 instead of 40.8506. The class is renamed, because {@code Paths} clashes
 * with the Pedro import, and {@link #start} is public for the auto. 01 is the same as {@link Red3Tips_1_0}; 02, 04 and 06 have
 * their own shot spots (farther up the field than the three-tip's). 05 picks up the near (Side)
 * FLOWER and 06 carries it back to a FAR shot spot, despite the exported names.
 */
public final class Red2Tips_1_0 {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(54.3656, 9.439, 270);
    private final Pose point101Garden = poseFactory.of(6.9632, 9.9654, 270);
    private final Pose point101GardenControl1 = poseFactory.of(37.7001, 12.3667, 0);
    private final Pose point101GardenControl2 = poseFactory.of(7.6473, 26.5441, 0);
    private final Pose point202Gardenloadfar = poseFactory.of(55.4026, 123.3594, 270);
    private final Pose point202GardenloadfarControl1 = poseFactory.of(36.8968, 7.2044, 0);
    private final Pose point202GardenloadfarControl2 = poseFactory.of(32.8276, 32.5069, 0);
    private final Pose point202GardenloadfarControl3 = poseFactory.of(16.8194, 115.9993, 0);
    private final Pose point303FarflowerStart = poseFactory.of(55.4026, 123.3594, 70);
    private final Pose point303Farflower = poseFactory.of(46.5317, 134.4926, 90);
    private final Pose point303FarflowerControl1 = poseFactory.of(46.7785, 115.6932, 0);
    private final Pose point404Flowerloadfar = poseFactory.of(36.9136, 123.0785, 90);
    private final Pose point404FlowerloadfarControl1 = poseFactory.of(46.4937, 125, 0);
    private final Pose point505NearFlowerStart = poseFactory.of(36.9136, 123.0785, 90);
    private final Pose point505NearFlower = poseFactory.of(9.608, 46.9503, 180);
    private final Pose point505NearFlowerControl1 = poseFactory.of(22.6382, 85.4818, 0);
    private final Pose point505NearFlowerControl2 = poseFactory.of(43.7826, 44.6638, 0);
    private final Pose point606FlowerloadnearStart = poseFactory.of(9.608, 46.9503, 180);
    private final Pose point606Flowerloadnear = poseFactory.of(49.0151, 122.846, 160);
    private final Pose point606FlowerloadnearControl1 = poseFactory.of(46.1101, 42.6882, 0);
    private final Pose point606FlowerloadnearControl2 = poseFactory.of(16.8431, 107.3396, 0);
    private final Pose point707Park = poseFactory.of(11.9513, 94.5223, 0.1336);
    private final Pose point707ParkControl1 = poseFactory.of(36.1682, 94.5788, 0);

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

    public Path path505NearFlower() {
        return Paths.curve(point505NearFlowerStart, point505NearFlowerControl1, point505NearFlowerControl2, point505NearFlower).linear(point505NearFlowerStart, point505NearFlower);
    }

    public Path path606Flowerloadnear() {
        return Paths.curve(point606FlowerloadnearStart, point606FlowerloadnearControl1, point606FlowerloadnearControl2, point606Flowerloadnear).linear(point606FlowerloadnearStart, point606Flowerloadnear);
    }

    public Path path707Park() {
        return Paths.curve(point606Flowerloadnear, point707ParkControl1, point707Park).reverseTangent();
    }
}
