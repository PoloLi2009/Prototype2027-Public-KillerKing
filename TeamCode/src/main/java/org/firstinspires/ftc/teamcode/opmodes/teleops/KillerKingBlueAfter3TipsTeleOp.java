package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.autos.Blue3TipsAuto;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.PinpointConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision.Alliance;

/**
 * Continue from Blue3Tips1.0: its measured end pose if fresh, else its preset park point. The same as
 * "Blue" otherwise ("Blue" starts at the Blue2Tips1.0 park, 90 deg off from this one).
 */
@TeleOp(name = "Blue After 3Tips", group = "Main")
public final class KillerKingBlueAfter3TipsTeleOp extends KillerKingTeleOp {
    public KillerKingBlueAfter3TipsTeleOp() {
        super(Alliance.BLUE, PinpointConstants.BLUE_AFTER_3TIPS_START_POSE, Blue3TipsAuto.NAME);
    }
}
