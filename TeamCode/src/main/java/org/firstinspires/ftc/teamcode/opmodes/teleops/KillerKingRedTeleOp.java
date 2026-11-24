package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision.Alliance;

@TeleOp(name = "Ba Red", group = "Test")
public final class KillerKingRedTeleOp extends KillerKingTeleOp {
    public KillerKingRedTeleOp() {
        super(Alliance.RED);
    }
}
