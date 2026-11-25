package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision.Alliance;

@TeleOp(name = "Ba Limelight Tuning", group = "Quest")
public final class KillerKingLimelightTuningTeleOp extends KillerKingTeleOp {
    public KillerKingLimelightTuningTeleOp() {
        super(Alliance.BLUE, true);
    }
}
