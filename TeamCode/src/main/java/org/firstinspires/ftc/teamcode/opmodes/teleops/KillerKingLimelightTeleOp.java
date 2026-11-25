package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision.Alliance;

@TeleOp(name = "Blue", group = "Main")
public final class KillerKingLimelightTeleOp extends KillerKingTeleOp {
    public KillerKingLimelightTeleOp() {
        super(Alliance.BLUE, false);
    }
}
