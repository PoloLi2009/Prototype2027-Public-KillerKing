package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision.Alliance;

@TeleOp(name = "Red", group = "Main")
public final class KillerKingRedLimelightTeleOp extends KillerKingTeleOp {
    public KillerKingRedLimelightTeleOp() {
        super(Alliance.RED, false);
    }
}
