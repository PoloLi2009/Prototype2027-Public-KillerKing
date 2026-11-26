package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;

/**
 * Pedro's Mecanum with the TeleOp's strafe yaw compensation: the front wheels' share of the strafe
 * is scaled by {@link DriveConstants#frontStrafeScale}. Pedro's normalization and saturation limit
 * both go through {@link #computeWheelPowersUnnormalized}, so overriding it covers both.
 */
public class CompensatedMecanum extends Mecanum {
    // Pedro's wheel order in computeWheelPowersUnnormalized.
    private static final int FL = 0;
    private static final int FR = 1;

    public CompensatedMecanum(HardwareMap map, MecanumConfig config) {
        super(map, config);
    }

    @Override
    public double[] computeWheelPowersUnnormalized(DrivePowers powers) {
        double[] wheels = super.computeWheelPowersUnnormalized(powers);
        // Pedro's strafe is left-positive: FL gets -strafe and FR +strafe. Shrink both toward zero.
        double removed = powers.strafe() * (1.0 - DriveConstants.frontStrafeScale);
        wheels[FL] += removed;
        wheels[FR] -= removed;
        return wheels;
    }
}
