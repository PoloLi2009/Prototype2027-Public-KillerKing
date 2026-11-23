package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.function.Supplier;

public final class TurretAimCommand extends CommandBase {
    private static final double REAR_DIRECTION_DEGREES = 180.0;
    private static final double TARGET_X_IN = 84.0;
    private static final double LOW_TARGET_Y_IN = 58.0;
    private static final double HIGH_TARGET_Y_IN = 96.0;
    private static final double SWITCH_Y_IN = 72.0;

    private final Supplier<Pose2D> poseSource;
    private final Turret turret;
    private boolean enabled;
    private double targetY = LOW_TARGET_Y_IN;
    private double relativeTargetDegrees;
    private boolean targetReachable;

    public TurretAimCommand(Supplier<Pose2D> poseSource, Turret turret) {
        this.poseSource = poseSource;
        this.turret = turret;
        addRequirements(turret);
    }

    public void toggleAim() {
        enabled = !enabled;
    }

    @Override
    public void execute() {
        if (!enabled) {
            if (turret.getOffsetDegrees() != 0.0) {
                turret.setOffsetDegrees(0);
            }
            return;
        }

        Pose2D pose = poseSource.get();
        if (pose == null) {
            return;
        }

        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        targetY = y < SWITCH_Y_IN ? LOW_TARGET_Y_IN : HIGH_TARGET_Y_IN;
        double fieldTargetDegrees = Math.toDegrees(Math.atan2(targetY - y, TARGET_X_IN - x));
        relativeTargetDegrees = wrapDegrees(fieldTargetDegrees - pose.getHeading(AngleUnit.DEGREES));
        double offsetDegrees = wrapDegrees(relativeTargetDegrees - REAR_DIRECTION_DEGREES);
        targetReachable = Math.abs(offsetDegrees) <= Turret.MAX_OFFSET_DEGREES;
        turret.setOffsetDegrees(offsetDegrees);
    }

    public void addTelemetry(Telemetry telemetry) {
        boolean hasPose = poseSource.get() != null;
        telemetry.addData("自瞄", enabled ? (hasPose ? "ON" : "等待位姿") : "OFF");
        telemetry.addData("云台舵机位置", "%.3f", turret.getPosition());
        if (enabled && hasPose) {
            double offset = turret.getOffsetDegrees();
            telemetry.addData("自瞄目标 (in)", "(%.0f, %.0f)", TARGET_X_IN, targetY);
            telemetry.addData("目标相对车头", "%.1f°", relativeTargetDegrees);
            telemetry.addData("云台相对车尾偏移", "%.1f° (限位 ±%.0f°)", offset, Turret.MAX_OFFSET_DEGREES);
            telemetry.addData("发射口实际朝向", "%.1f°", wrapDegrees(REAR_DIRECTION_DEGREES + offset));
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
