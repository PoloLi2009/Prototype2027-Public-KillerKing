package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;

/**
 * Pedro AutoTune procedures, copied from the Overturn robot. Run them in order (Mecanum, Pinpoint,
 * Foresight) and paste each export into {@link Constants}. Overturn's dashboard-based "Tests"
 * procedure is left out because Killer King does not include FTC Dashboard.
 */
public class Tuning {
    @Tuner
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresight() {
        return new ForesightTuner(
                hardwareMap -> new PinpointLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new CompensatedMecanum(hardwareMap, Constants.drivetrainConfig));
    }
}
