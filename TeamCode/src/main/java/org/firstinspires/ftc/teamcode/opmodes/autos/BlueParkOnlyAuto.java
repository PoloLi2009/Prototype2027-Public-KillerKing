package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Blue park only: {@link RedParkOnlyAuto} rotated 180 deg about the field center (x' = 141.5 - x,
 * y' = 141.5 - y, heading + 180). Wait 25 s, then one line from (131.9772, 53.6726) at 270 deg to
 * (129.5669, 46.9963) at 180 deg, the Blue2Tips1.0 park. Preselects the default "Blue" TeleOp.
 */
@Autonomous(name = "BlueParkOnly", group = "Auto", preselectTeleOp = "Blue")
public final class BlueParkOnlyAuto extends ParkOnlyAuto {
    private static final PoseFactory POSES = PoseFactory.degrees();

    public BlueParkOnlyAuto() {
        super("BlueParkOnly", POSES.of(131.9772, 53.6726, 270), POSES.of(129.5669, 46.9963, 180));
    }
}
