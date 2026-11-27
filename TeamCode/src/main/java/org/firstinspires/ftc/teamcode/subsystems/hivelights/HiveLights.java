package org.firstinspires.ftc.teamcode.subsystems.hivelights;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

public final class HiveLights extends SubsystemBase {
    private final Servo firstLight;
    private final Servo secondLight;
    private double currentPosition = Double.NaN;

    public HiveLights(HardwareMap hardwareMap) {
        firstLight = hardwareMap.tryGet(Servo.class, HiveLightsConstants.FIRST_LIGHT_NAME);
        secondLight = hardwareMap.tryGet(Servo.class, HiveLightsConstants.SECOND_LIGHT_NAME);
        turnOff();
    }

    /**
     * Red (steady) while the turret target is in its dead zone; otherwise green inside the shooting
     * distance range and red outside it.
     */
    public void showDistance(Pose2d rawOdometry, boolean blueAlliance, boolean turretDeadZone) {
        if (turretDeadZone) {
            setBoth(HiveLightsConstants.RED_POSITION);
            return;
        }
        double distanceCm = Double.NaN;
        if (rawOdometry != null) {
            double x = DistanceUnit.INCH.fromMeters(rawOdometry.getX());
            double y = DistanceUnit.INCH.fromMeters(rawOdometry.getY());
            double targetX = blueAlliance
                    ? TurretConstants.BLUE_TARGET_X_INCHES : TurretConstants.RED_TARGET_X_INCHES;
            double targetY = y < TurretConstants.SWITCH_Y_INCHES
                    ? TurretConstants.LOW_TARGET_Y_INCHES : TurretConstants.HIGH_TARGET_Y_INCHES;
            distanceCm = DistanceUnit.CM.fromInches(Math.hypot(targetX - x, targetY - y));
        }
        boolean inRange = distanceCm >= HiveLightsConstants.MIN_GREEN_DISTANCE_CM
                && distanceCm <= HiveLightsConstants.MAX_GREEN_DISTANCE_CM;
        setBoth(inRange ? HiveLightsConstants.GREEN_POSITION : HiveLightsConstants.RED_POSITION);
    }

    public void turnOff() {
        setBoth(HiveLightsConstants.OFF_POSITION);
    }

    private void setBoth(double position) {
        if (currentPosition == position) {
            return;
        }
        if (firstLight != null) {
            firstLight.setPosition(position);
        }
        if (secondLight != null) {
            secondLight.setPosition(position);
        }
        currentPosition = position;
    }
}
