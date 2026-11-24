package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightVision.Alliance;

@TeleOp(name = "Vision Fusion Command Test - Blue", group = "Test")
public final class BlueVisionFusionCommandTest extends VisionFusionCommandTest {
    public BlueVisionFusionCommandTest() {
        super(Alliance.BLUE);
    }
}
