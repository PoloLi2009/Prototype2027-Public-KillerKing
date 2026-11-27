package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.autos.Red3Tips2Auto;
import org.firstinspires.ftc.teamcode.opmodes.autos.Red3Tips3Auto;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.AutoEndPose;
import org.firstinspires.ftc.teamcode.subsystems.pinpoint.PinpointConstants;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision.Alliance;

/** Continue from the latest red three-tip push, or the preset push endpoint. */
@TeleOp(name = "Red After 3Tips", group = "Main")
public final class KillerKingRedAfter3TipsTeleOp extends KillerKingTeleOp {
    public KillerKingRedAfter3TipsTeleOp() {
        super(Alliance.RED, PinpointConstants.RED_AFTER_3TIPS_START_POSE, Red3Tips2Auto.NAME);
    }

    @Override
    protected String previousAutonomous() {
        return AutoEndPose.recent(Red3Tips3Auto.NAME) != null ? Red3Tips3Auto.NAME : super.previousAutonomous();
    }
}
