package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Red park only (Pedro Visualizer export): wait 25 s, then one line from (9.5228, 87.8274) at 90 deg
 * to (11.9331, 94.5037) at 0 deg, the Red2Tips1.0 park. Preselects the default "Red" TeleOp.
 */
@Autonomous(name = "RedParkOnly", group = "Auto", preselectTeleOp = "Red")
public final class RedParkOnlyAuto extends ParkOnlyAuto {
    private static final PoseFactory POSES = PoseFactory.degrees();

    public RedParkOnlyAuto() {
        super("RedParkOnly", POSES.of(9.5228, 87.8274, 90), POSES.of(11.9331, 94.5037, 0));
    }
}
