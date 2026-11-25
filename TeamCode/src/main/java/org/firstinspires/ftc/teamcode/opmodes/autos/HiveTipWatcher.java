package org.firstinspires.ftc.teamcode.opmodes.autos;

import org.firstinspires.ftc.teamcode.subsystems.localization.VisionLocalization.HiveMotionState;

/**
 * Tells whether one volley tipped the HIVE, from the Limelight tilt alone.
 *
 * <p>It compares sides, not angles: the side the HIVE rests on when the gate opens against any side
 * it rests on afterwards. A TIP always swaps the resting side, so this needs no calibration of which
 * tilt sign means which CELL. All times are {@code System.nanoTime() / 1e9}, like the vision code.
 */
final class HiveTipWatcher {
    enum Result { TIPPED, NOT_TIPPED, UNKNOWN }

    // Same "resting on a side" threshold as VisionLocalization's motion detector.
    static final double MIN_SIDE_DEGREES = 3.0;
    // A side counts only after STILL readings agree on it for this long.
    static final double CONFIRM_SECONDS = 0.10;
    // The pre-volley side must have been seen this recently when the gate opens.
    static final double BASELINE_MAX_AGE_SECONDS = 2.0;
    // After the feed stops, allow ball flight and the HIVE's swing before "same side" means no TIP.
    static final double SETTLE_SECONDS = 0.6;

    private int side;                                   // latest resting side, -1 / +1, 0 = none yet
    private double sideSince = Double.NaN;              // first STILL reading of that side
    private double sideConfirmedAt = Double.NEGATIVE_INFINITY;

    private boolean armed;
    private int baseline;
    private double volleyEnd = Double.NaN;
    private boolean flipped;
    private boolean sameSideAfterSettle;

    /** Call every loop with VisionLocalization's tilt (NaN when stale) and motion state. */
    void update(double now, double tiltDegrees, HiveMotionState state) {
        if (state != HiveMotionState.STILL || !Double.isFinite(tiltDegrees)
                || Math.abs(tiltDegrees) < MIN_SIDE_DEGREES) {
            return;
        }
        int reading = tiltDegrees > 0 ? 1 : -1;
        if (reading != side) {
            side = reading;
            sideSince = now;
        }
        if (now - sideSince < CONFIRM_SECONDS) return;
        sideConfirmedAt = now;
        if (!armed || baseline == 0) return;
        if (side != baseline) {
            flipped = true;                             // a TIP cannot be undone without 8 more balls
        } else if (Double.isFinite(volleyEnd) && now >= volleyEnd + SETTLE_SECONDS) {
            sameSideAfterSettle = true;
        }
    }

    /** Call when the gate opens, before any ball of the volley can reach the CELL. */
    void arm(double now) {
        armed = true;
        baseline = now - sideConfirmedAt <= BASELINE_MAX_AGE_SECONDS ? side : 0;
        volleyEnd = Double.NaN;
        flipped = false;
        sameSideAfterSettle = false;
    }

    /** Call when the feed stops. */
    void volleyEnded(double now) {
        volleyEnd = now;
    }

    Result result() {
        if (!armed || baseline == 0) return Result.UNKNOWN;
        if (flipped) return Result.TIPPED;
        return sameSideAfterSettle ? Result.NOT_TIPPED : Result.UNKNOWN;
    }

    String describe() {
        if (!armed) return "not armed";
        if (baseline == 0) return "no HIVE side seen before the volley";
        return String.format("before %+d, latest %+d%s", baseline, side,
                flipped ? ", flipped" : sameSideAfterSettle ? ", same side after settle" : "");
    }
}
