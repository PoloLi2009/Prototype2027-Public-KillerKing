package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision.Alliance;

@TeleOp(name = "Vision Fusion Command Test - Red", group = "Test")
public final class RedVisionFusionCommandTest extends VisionFusionCommandTest {
    public RedVisionFusionCommandTest() {
        super(Alliance.RED);
    }
}
