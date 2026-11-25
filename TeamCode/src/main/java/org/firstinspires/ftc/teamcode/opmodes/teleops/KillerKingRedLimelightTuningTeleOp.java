package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision.Alliance;

@TeleOp(name = "Ba Red Limelight Tuning", group = "Quest")
public final class KillerKingRedLimelightTuningTeleOp extends KillerKingTeleOp {
    public KillerKingRedLimelightTuningTeleOp() {
        super(Alliance.RED, true);
    }
}
