package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public final class TurretAimCommand extends CommandBase {
    private final Supplier<Pose2D> poseSource;
    private final BooleanSupplier blueAlliance;
    private final Turret turret;
    private boolean enabled = true;
    private double targetX = TurretConstants.BLUE_TARGET_X_INCHES;
    private double targetY = TurretConstants.LOW_TARGET_Y_INCHES;
    private double targetDistanceCm = Double.NaN;
    private double relativeTargetDegrees;
    private boolean targetReachable = true;
    private boolean hasPose;

    public TurretAimCommand(Supplier<Pose2D> poseSource, BooleanSupplier blueAlliance, Turret turret) {
        this.poseSource = poseSource;
        this.blueAlliance = blueAlliance;
        this.turret = turret;
        addRequirements(turret);
    }

    public void toggleAim() {
        enabled = !enabled;
    }

    /** Sets aim on or off regardless of the default; disabled aim re-centers the turret. */
    public void setAimEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    @Override
    public void execute() {
        if (!enabled && turret.getOffsetDegrees() != 0.0) {
            turret.setOffsetDegrees(0);
        }

        Pose2D pose = poseSource.get();
        hasPose = pose != null;
        if (pose == null) {
            targetDistanceCm = Double.NaN;
            return;
        }

        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        targetX = blueAlliance.getAsBoolean()
                ? TurretConstants.BLUE_TARGET_X_INCHES : TurretConstants.RED_TARGET_X_INCHES;
        targetY = y < TurretConstants.SWITCH_Y_INCHES
                ? TurretConstants.LOW_TARGET_Y_INCHES : TurretConstants.HIGH_TARGET_Y_INCHES;
        double deltaX = targetX - x;
        double deltaY = targetY - y;
        targetDistanceCm = DistanceUnit.CM.fromInches(Math.hypot(deltaX, deltaY));

        if (!enabled) {
            targetReachable = true;
            return;
        }

        double fieldTargetDegrees = Math.toDegrees(Math.atan2(deltaY, deltaX));
        relativeTargetDegrees = wrapDegrees(fieldTargetDegrees - pose.getHeading(AngleUnit.DEGREES));
        double offsetDegrees = wrapDegrees(relativeTargetDegrees - TurretConstants.CENTER_FACING_DEGREES);
        targetReachable = Math.abs(offsetDegrees) <= TurretConstants.MAX_OFFSET_DEGREES;
        turret.setOffsetDegrees(offsetDegrees);
    }

    public double getTargetDistanceCm() {
        return targetDistanceCm;
    }

    /**
     * True while aiming is on, there is a pose, and the target lies in the turret's dead zone: outside
     * the +/- MAX_OFFSET_DEGREES travel, so the turret stops at its limit instead of on the target.
     */
    public boolean isInDeadZone() {
        return enabled && hasPose && !targetReachable;
    }

    public void addTelemetry(Telemetry telemetry) {
        boolean hasPose = poseSource.get() != null;
        telemetry.addData("自瞄", enabled ? (hasPose ? "ON" : "等待位姿") : "OFF");
        telemetry.addData("云台舵机位置", "%.3f", turret.getPosition());
        if (enabled && hasPose) {
            double offset = turret.getOffsetDegrees();
            telemetry.addData("自瞄目标 (in)", "(%.0f, %.0f)", targetX, targetY);
            telemetry.addData("目标相对车头", "%.1f°", relativeTargetDegrees);
            telemetry.addData("云台相对中位偏移", "%.1f° (限位 ±%.0f°)",
                    offset, TurretConstants.MAX_OFFSET_DEGREES);
            telemetry.addData("发射口指令朝向", "%.1f°",
                    wrapDegrees(TurretConstants.CENTER_FACING_DEGREES + offset));
            telemetry.addData("目标可达", targetReachable ? "是" : "超出行程");
        }
    }

    private static double wrapDegrees(double angle) {
        double wrapped = angle % 360;
        if (wrapped > 180) {
            return wrapped - 360;
        }
        if (wrapped < -180) {
            return wrapped + 360;
        }
        return wrapped;
    }
}
