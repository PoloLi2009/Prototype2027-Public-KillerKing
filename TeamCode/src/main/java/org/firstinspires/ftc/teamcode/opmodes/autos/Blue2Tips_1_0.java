package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Pedro route for Blue2Tips1.0: {@link Red2Tips_1_0} mirrored to the blue side.
 * Every pose is the red one rotated 180 deg about the field center (point symmetry, Pedro field
 * 141.5 in): x' = 141.5 - x, y' = 141.5 - y, heading + 180 deg (control points keep their exported 0;
 * their heading is unused). The path methods are the red ones unchanged. Generated from the red
 * route, not a Visualizer export: change the red route and regenerate, so both stay mirrored.
 */
public final class Blue2Tips_1_0 {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(87.1344, 132.061, 90);
    private final Pose point101Garden = poseFactory.of(134.5368, 131.5346, 90);
    private final Pose point101GardenControl1 = poseFactory.of(103.7999, 129.1333, 0);
    private final Pose point101GardenControl2 = poseFactory.of(133.8527, 114.9559, 0);
    private final Pose point202Gardenloadfar = poseFactory.of(86.0974, 18.1406, 90);
    private final Pose point202GardenloadfarControl1 = poseFactory.of(104.6032, 134.2956, 0);
    private final Pose point202GardenloadfarControl2 = poseFactory.of(108.6724, 108.9931, 0);
    private final Pose point202GardenloadfarControl3 = poseFactory.of(124.6806, 25.5007, 0);
    private final Pose point303FarflowerStart = poseFactory.of(86.0974, 18.1406, 250);
    private final Pose point303Farflower = poseFactory.of(94.9683, 7.0074, 270);
    private final Pose point303FarflowerControl1 = poseFactory.of(94.7215, 25.8068, 0);
    private final Pose point404Flowerloadfar = poseFactory.of(104.5864, 18.4215, 270);
    private final Pose point404FlowerloadfarControl1 = poseFactory.of(95.0063, 16.5, 0);
    private final Pose point505NearFlowerStart = poseFactory.of(104.5864, 18.4215, 270);
    private final Pose point505NearFlower = poseFactory.of(131.892, 94.5497, 0);
    private final Pose point505NearFlowerControl1 = poseFactory.of(118.8618, 56.0182, 0);
    private final Pose point505NearFlowerControl2 = poseFactory.of(97.7174, 96.8362, 0);
    private final Pose point606FlowerloadnearStart = poseFactory.of(131.892, 94.5497, 0);
    private final Pose point606Flowerloadnear = poseFactory.of(92.4849, 18.654, 340);
    private final Pose point606FlowerloadnearControl1 = poseFactory.of(95.3899, 98.8118, 0);
    private final Pose point606FlowerloadnearControl2 = poseFactory.of(124.6569, 34.1604, 0);
    private final Pose point707Park = poseFactory.of(129.5487, 46.9777, 180.1336);
    private final Pose point707ParkControl1 = poseFactory.of(105.3318, 46.9212, 0);

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
