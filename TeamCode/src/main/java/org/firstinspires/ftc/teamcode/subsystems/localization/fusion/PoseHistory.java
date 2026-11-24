package org.firstinspires.ftc.teamcode.subsystems.localization.fusion;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import java.util.Map;
import java.util.TreeMap;

/** Host-monotonic timestamps, meters, radians; turret angles are unwrapped degrees. */
public class PoseHistory {
    public static final double DURATION_SECONDS = 1.5;
    public static final class Sample {
        public final double time, turretDegrees;
        public final double turretTrackingErrorDegrees;
        public final Pose2d odometry;
        private Sample(double time, Pose2d odometry, double turretDegrees, double turretTrackingErrorDegrees) {
            this.time = time;
            this.odometry = odometry;
            this.turretDegrees = turretDegrees;
            this.turretTrackingErrorDegrees = turretTrackingErrorDegrees;
        }
    }
    private final TreeMap<Double, Sample> samples = new TreeMap<>();
    public void clear() { samples.clear(); }
    public void add(double time, Pose2d odometry, double turretDegrees) {
        add(time, odometry, turretDegrees, 0);
    }
    public void add(double time, Pose2d odometry, double turretDegrees, double turretTrackingErrorDegrees) {
        samples.put(time, new Sample(time, odometry, turretDegrees, turretTrackingErrorDegrees));
        while (samples.size() > 2 && samples.firstKey() < time - DURATION_SECONDS)
            samples.pollFirstEntry();
    }
    public Sample sample(double time) {
        Map.Entry<Double, Sample> a = samples.floorEntry(time), b = samples.ceilingEntry(time);
        if (a == null || b == null) return null; // Never silently clamp an old/future measurement.
        if (a.getKey().equals(b.getKey())) return a.getValue();
        double t = (time - a.getKey()) / (b.getKey() - a.getKey());
        Twist2d twist = a.getValue().odometry.log(b.getValue().odometry);
        return new Sample(time, a.getValue().odometry.exp(new Twist2d(
                twist.dx * t, twist.dy * t, twist.dtheta * t)),
                a.getValue().turretDegrees + t * (b.getValue().turretDegrees - a.getValue().turretDegrees),
                Math.max(a.getValue().turretTrackingErrorDegrees, b.getValue().turretTrackingErrorDegrees));
    }
}
