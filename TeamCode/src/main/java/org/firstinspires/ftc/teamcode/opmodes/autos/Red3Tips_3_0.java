package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** Red-side prototype. Pedro inches, intake forward. The alliance wall is x=0. */
public final class Red3Tips_3_0 {
    private static final PoseFactory P = PoseFactory.degrees();
    private final Red2Tips_1_0 twoTip = new Red2Tips_1_0();
    public final Pose start = twoTip.start;
    final Pose garden = P.of(6.9632, 9.9654, 270);
    final Pose gardenShot = P.of(46.5, 121.8, 90);
    final Pose farFlower = P.of(46.53, 134.49, 90);
    final Pose farShot = P.of(34.7, 122.7, 90);
    final Pose partner = P.of(8, 103.4, 270);
    final Pose partnerShot = P.of(33.2, 26.8, 270);
    final Pose sideFlower = P.of(9.035, 46.6183, 180);
    final Pose sideShot = P.of(27.7, 32.1, 180);
    final Pose square = P.of(8, 64, 270);
    public final Pose end = P.of(8, 88.5, 270);

    Path gardenPickup() {
        return twoTip.path101Garden();
    }

    // Turn while traversing the open x=30.5 lane. Arrive already facing the Far flower.
    Path gardenToShot(Pose from) {
        Pose lane = P.of(30.5, 42, 270), top = P.of(30.5, 98, 90);
        return Paths.path(Paths.curve(from, P.of(25, 10, 0), P.of(30.5, 22, 0), lane).constant(lane),
                Paths.line(lane, top).linear(lane, top),
                Paths.curve(top, P.of(30.5, 110, 0), P.of(46.5, 114, 0), gardenShot).constant(gardenShot));
    }

    Path farPickup(Pose from) {
        return cap(Paths.line(from, farFlower).constant(farFlower), 35);
    }

    // Leave each FLOWER on its own axis before turning away; start at measured contact, not beyond the wall.
    Path farExit(Pose from) {
        return Paths.curve(from, P.of(from.x(), 116, 0), P.of(40, 121, 0), farShot).constant(farShot);
    }

    Path partnerPickup(Pose from) {
        Pose midway = P.of(23.5, 122, 10), aligned = P.of(12, 115, 270);
        return Paths.path(cap(Paths.curve(from, P.of(30, 123, 0), midway).linear(from, midway), 24),
                cap(Paths.curve(midway, P.of(19, 122, 0), P.of(14, 118, 0), aligned)
                        .linear(midway, aligned), 24),
                Paths.curve(aligned, P.of(9, 112, 0), P.of(8, 107, 0), partner).constant(partner));
    }

    Path partnerExit(Pose from) {
        return Paths.curve(from, P.of(from.x(), 119, 0), P.of(44, 102, 0),
                P.of(34, 103, 0), partnerShot).constant(partnerShot);
    }

    Path sidePickup(Pose from) {
        Pose axis = P.of(22, 46.6183, 180);
        return Paths.path(Paths.curve(from, P.of(31, 39, 0), P.of(27, 46.6183, 0), axis).linear(from, axis),
                cap(Paths.line(axis, sideFlower).constant(sideFlower), 35));
    }

    Path sideExit(Pose from) {
        return Paths.curve(from, P.of(28, from.y(), 0), P.of(26, 36, 0), sideShot).constant(sideShot);
    }

    Path lineUp(Pose from) {
        Pose turned = P.of(19, 56, 270);
        return Paths.path(Paths.curve(from, P.of(30, 44, 0), P.of(24, 52, 0), turned).linear(from, turned),
                cap(Paths.curve(turned, P.of(14, 59, 0), P.of(8, 60, 0), square).constant(square), 22));
    }

    Path push() {
        return cap(Paths.line(square, end).constant(end), 20);
    }

    private static Path cap(Path path, double inchesPerSecond) {
        return path.with(Constants.foresightConfig.maxVelocityConstraint.at(inchesPerSecond));
    }
}
