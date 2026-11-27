package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterConstants;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.math.BigDecimal;
import java.math.MathContext;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.function.BiConsumer;

/**
 * Measures the flywheel and picks KP / KF for the Shooter PIDF (KI and KD stay 0). Take every ball out
 * first: only the flywheel turns. About 2 minutes.
 *
 * <ol>
 * <li>Open loop: holds each power in OPEN_LOOP_POWERS until the flywheel settles, fits
 *     steady RPM = rpmPerPower * power + intercept, the first-order time constant of each step and
 *     the encoder noise. KF is the power needed at the test RPM divided by that RPM. The Shooter
 *     scales its output by NOMINAL_VOLTAGE / battery voltage, so rpmPerPower and KF are converted
 *     from the battery voltage of the open-loop steps to ShooterConstants.NOMINAL_VOLTAGE.</li>
 * <li>KP: runs the real Shooter controller at the test RPM for each candidate (KI = KD = 0, KF from 1).
 *     Each candidate: the steady RPM jitter, then DIPS_PER_CANDIDATE dips (braking at MIN_POWER until
 *     DIP_RPM is lost, as a ball would take it) and the RPM error SCORE_FROM_S-SCORE_TO_S after each
 *     dip, when the next ball of a volley arrives. Score = that error + the jitter. The current
 *     ShooterConstants gains run first as the baseline.</li>
 * <li>KI: with the best KP, runs the KI candidates for reference only. KI stays 0 (user, 2026-11-27:
 *     the integral overshoots after every ball); voltage compensation makes up for the battery.</li>
 * </ol>
 *
 * Set the loop period (D-pad up / down in INIT) to the TeleOp loop time ("循环时间" in its
 * telemetry): the same gains behave differently at 10 ms and at 40 ms per loop.
 *
 * Every loop is written to a CSV in FIRST/ShooterAutoTuner on the Robot Controller (columns in
 * CSV_HEADER, results and the report as lines starting with #), so the run can be analysed again
 * off the robot.
 */
@TeleOp(name = "Shooter Auto Tuner", group = "Test")
public class ShooterAutoTuner extends LinearOpMode {
    private static final String TAG = "ShooterAutoTuner";

    // 1. Open loop. Each power is held for 5 time constants (the first one from rest for
    // FIRST_SETTLE_S), then sampled for SAMPLE_S.
    static final double[] OPEN_LOOP_POWERS = {0.35, 0.45, 0.55, 0.65, 0.75, 0.85};
    static final double FIRST_SETTLE_S = 3.0;
    static final double MIN_SETTLE_S = 1.5;
    static final double MAX_SETTLE_S = 5.0;
    static final double SAMPLE_S = 1.0;
    static final double FIT_WINDOW_S = 1.5;
    static final double MIN_RPM_READING = 100;

    // 2. KP candidates as loop gains, KP * rpmPerPower: 2 is soft, 35 is stiff.
    static final double[] LOOP_GAINS = {2, 4, 6, 9, 13, 18, 25, 35};
    // 3. KI candidates as integral times KP / KI, slow to fast.
    static final double[] INTEGRAL_TIMES_S = {2.0, 1.0, 0.5};

    static final double SPIN_UP_S = 2.0;
    static final double CLOSED_SETTLE_S = 1.5;
    static final int DIPS_PER_CANDIDATE = 3;
    // RPM one ball is assumed to take. Results are scaled to exactly this dip.
    static final double DIP_RPM = 150;
    static final double MAX_DIP_S = 0.5;
    static final double RECORD_S = 0.8;
    static final double BETWEEN_DIPS_S = 0.6;
    // In a TeleOp volley the next ball arrives about 0.2-0.3 s after the previous one.
    static final double SCORE_FROM_S = 0.15;
    static final double SCORE_TO_S = 0.40;
    static final double RECOVERED_RPM = 25;
    // Just over one 35 RPM step of the distance table.
    static final double OVERSHOOT_LIMIT_RPM = 40;
    static final double POWER_HEADROOM_WARNING = 0.85;

    static final double DEFAULT_LOOP_MS = 20;
    static final double LOOP_MS_STEP = 5;
    static final double MAX_LOOP_MS = 60;
    static final double TEST_RPM_STEP = 100;
    static final double MIN_TEST_RPM = 1000;

    private Shooter shooter;
    private List<VoltageSensor> voltageSensors;
    private double loopMs = DEFAULT_LOOP_MS;
    private double testRpm = ShooterConstants.INITIAL_RPM;
    private long startNs;
    private long nextLoopNs;
    private long lastLoopNs;
    private double loopMsSum;
    private int loopCount;
    private double loopMsMax;
    private double lastTelemetryS = Double.NEGATIVE_INFINITY;
    private String failure;
    private final List<String> resultLines = new ArrayList<>();

    static final String CSV_FOLDER = "ShooterAutoTuner";
    static final String CSV_HEADER = "t_s,section,phase,power,raw_rpm,filtered_rpm,target_rpm";
    private BufferedWriter csv;
    private File csvFile;
    private String section = "";

    @Override
    public void runOpMode() {
        shooter = new Shooter(hardwareMap);
        voltageSensors = hardwareMap.getAll(VoltageSensor.class);
        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;
        while (opModeInInit()) {
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
            if (up && !lastUp) loopMs = Math.min(MAX_LOOP_MS, loopMs + LOOP_MS_STEP);
            if (down && !lastDown) loopMs = Math.max(0, loopMs - LOOP_MS_STEP);
            if (right && !lastRight) testRpm = Math.min(ShooterConstants.MAX_RPM, testRpm + TEST_RPM_STEP);
            if (left && !lastLeft) testRpm = Math.max(MIN_TEST_RPM, testRpm - TEST_RPM_STEP);
            lastUp = up;
            lastDown = down;
            lastLeft = left;
            lastRight = right;

            telemetry.addLine("飞轮自动调参：只转飞轮，车、Intake、球挡都不动");
            telemetry.addLine("开始前取出所有球。大约 2 分钟，随时按 STOP 停止。");
            telemetry.addData("循环周期（↑/↓）", "%.0f ms", loopMs);
            telemetry.addLine("  设成 TeleOp 遥测里\"循环时间\"的平均值，0 = 不限速");
            telemetry.addData("测试转速（←/→）", "%.0f RPM", testRpm);
            telemetry.addData("当前常数", "KP %s  KI %s  KD %s  KF %s", sig(ShooterConstants.KP),
                    sig(ShooterConstants.KI), sig(ShooterConstants.KD), sig(ShooterConstants.KF));
            telemetry.addData("电池", "%.2f V", readVoltage());
            telemetry.addLine("每一圈的数据会存到主控的 FIRST/" + CSV_FOLDER + "/");
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) {
            return;
        }

        startNs = System.nanoTime();
        nextLoopNs = startNs;
        openCsv();
        try {
            runTuner();
        } finally {
            shooter.setPowerOverride(Double.NaN);
            shooter.stop();
            closeCsv();
        }
    }

    private void runTuner() {
        double startVolts = readVoltage();
        OpenLoop open = measureOpenLoop();
        if (open == null) {
            if (failure != null) {
                note("failure " + failure);
                flushCsv();
                holdScreen(Collections.singletonList("停止：" + failure));
            }
            return;
        }

        // Spin up open loop to the test RPM, so the first candidate does not start from 0.85 power.
        shooter.setOpenLoopPower(open.powerAtTest);
        section = "spin up";
        double spinStart = now();
        while (now() - spinStart < SPIN_UP_S) {
            if (!tick()) return;
            double rpm = shooter.readRawRpm();
            record("open", open.powerAtTest, rpm, Double.NaN, Double.NaN);
            show("2/3 预转到测试转速", rpm, open.powerAtTest);
        }

        int total = 1 + LOOP_GAINS.length + INTEGRAL_TIMES_S.length;
        int index = 1;
        Candidate baseline = testCandidate("当前常数", index++, total, ShooterConstants.KP,
                ShooterConstants.KI, ShooterConstants.KD, ShooterConstants.KF);
        if (baseline == null) return;

        List<Candidate> kpRuns = new ArrayList<>();
        for (double loopGain : LOOP_GAINS) {
            // Loop gain at NOMINAL_VOLTAGE: the Shooter's voltage compensation keeps it there.
            double kp = loopGain / open.rpmPerPowerNominal;
            Candidate c = testCandidate("KP " + sig(kp), index++, total, kp, 0, 0, open.kf);
            if (c == null) return;
            kpRuns.add(c);
        }
        Candidate bestKp = pickBest(kpRuns);
        // KI stays 0 (the battery drop is compensated by voltage); the KI candidates are only run so
        // the report shows what an integral would do.
        for (double integralTime : INTEGRAL_TIMES_S) {
            double ki = bestKp.kp / integralTime;
            Candidate c = testCandidate("KI " + sig(ki) + "（参考）", index++, total, bestKp.kp, ki, 0, open.kf);
            if (c == null) return;
        }

        shooter.stop();
        holdScreen(report(open, baseline, bestKp, bestKp, kpRuns, startVolts, readVoltage()));
    }

    // ---------------------------------------------------------------- open loop

    private static final class OpenLoop {
        double rpmPerPower;             // at the battery voltage of the open-loop steps
        double rpmPerPowerNominal;      // the same at ShooterConstants.NOMINAL_VOLTAGE
        double volts = Double.NaN;      // mean battery voltage of the open-loop steps
        double intercept;
        double rSquared;
        double powerAtTest;             // open-loop power at the battery voltage of the steps
        double kf;                      // at NOMINAL_VOLTAGE, as the Shooter uses it
        double tauS = Double.NaN;
        double deadTimeS = Double.NaN;
        double noiseRpm;
    }

    private OpenLoop measureOpenLoop() {
        int n = OPEN_LOOP_POWERS.length;
        double[] steadyRpm = new double[n];
        double[] noise = new double[n];
        List<Double> taus = new ArrayList<>();
        List<Double> deadTimes = new ArrayList<>();
        double tauEstimate = Double.NaN;
        double previousRpm = 0;               // the flywheel starts at rest
        Stats volts = new Stats();
        for (int i = 0; i < n; i++) {
            double power = OPEN_LOOP_POWERS[i];
            shooter.setOpenLoopPower(power);
            double settleS = Double.isNaN(tauEstimate) ? FIRST_SETTLE_S
                    : Range.clip(5 * tauEstimate, MIN_SETTLE_S, MAX_SETTLE_S);
            String status = String.format("1/3 开环 %d/%d：功率 %.2f", i + 1, n, power);
            section = String.format("open %.2f", power);
            double stepStart = now();
            List<double[]> response = new ArrayList<>();
            while (true) {
                if (!tick()) return null;
                double t = now() - stepStart;
                double rpm = shooter.readRawRpm();
                record("step", power, rpm, Double.NaN, Double.NaN);
                if (t <= FIT_WINDOW_S) response.add(new double[]{t, rpm});
                if (t >= settleS) break;
                show(String.format("%s，等稳定 %.1f / %.1f s", status, t, settleS), rpm, power);
            }
            Stats sample = new Stats();
            double sampleStart = now();
            while (now() - sampleStart < SAMPLE_S) {
                if (!tick()) return null;
                double rpm = shooter.readRawRpm();
                record("sample", power, rpm, Double.NaN, Double.NaN);
                sample.add(rpm);
                show(status + "，采样", rpm, power);
            }
            steadyRpm[i] = sample.mean();
            noise[i] = sample.std();
            if (steadyRpm[i] < MIN_RPM_READING) {
                failure = String.format("功率 %.2f 时只读到 %.0f RPM。检查飞轮电机 %s 的编码器线和方向。",
                        power, steadyRpm[i], ShooterConstants.ENCODER_MOTOR_NAME);
                return null;
            }
            double[] fit = fitFirstOrder(response, previousRpm, steadyRpm[i]);
            if (fit != null) {
                if (i > 0) {                  // from rest, static friction distorts the first step
                    taus.add(fit[0]);
                    deadTimes.add(fit[1]);
                }
                tauEstimate = taus.isEmpty() ? fit[0] : median(taus);
            }
            double levelVolts = readVoltage();
            if (Double.isFinite(levelVolts)) volts.add(levelVolts);
            String levelLine = String.format(Locale.US,
                    "open loop %.2f: %.1f RPM, noise %.1f RPM, tau %s s, dead %s s, %.2f V",
                    power, steadyRpm[i], noise[i], fit == null ? "-" : sig(fit[0]),
                    fit == null ? "-" : sig(fit[1]), levelVolts);
            RobotLog.ii(TAG, "%s", levelLine);
            note(levelLine);
            previousRpm = steadyRpm[i];
        }

        // Least squares: steady RPM = rpmPerPower * power + intercept.
        double meanP = 0, meanR = 0;
        for (int i = 0; i < n; i++) {
            meanP += OPEN_LOOP_POWERS[i] / n;
            meanR += steadyRpm[i] / n;
        }
        double sxy = 0, sxx = 0, syy = 0;
        for (int i = 0; i < n; i++) {
            double dp = OPEN_LOOP_POWERS[i] - meanP, dr = steadyRpm[i] - meanR;
            sxy += dp * dr;
            sxx += dp * dp;
            syy += dr * dr;
        }
        OpenLoop open = new OpenLoop();
        open.rpmPerPower = sxy / sxx;
        open.intercept = meanR - open.rpmPerPower * meanP;
        open.rSquared = syy > 0 ? sxy * sxy / (sxx * syy) : 0;
        if (!(open.rpmPerPower > 0)) {
            failure = "功率越大转速反而越低，检查飞轮编码器方向。";
            return null;
        }
        open.powerAtTest = (testRpm - open.intercept) / open.rpmPerPower;
        if (open.powerAtTest >= ShooterConstants.MAX_POWER) {
            failure = String.format("%.0f RPM 需要功率 %.2f，超出飞轮能力。把测试转速调低。",
                    testRpm, open.powerAtTest);
            return null;
        }
        // The Shooter multiplies its output by NOMINAL_VOLTAGE / battery voltage, so its KF and KP are
        // for NOMINAL_VOLTAGE: the same RPM takes power * volts / NOMINAL_VOLTAGE there. Without a
        // voltage reading the values are used as measured.
        double voltsRatio = 1.0;
        if (volts.count() > 0) {
            open.volts = volts.mean();
            voltsRatio = open.volts / ShooterConstants.NOMINAL_VOLTAGE;
        }
        open.rpmPerPowerNominal = open.rpmPerPower / voltsRatio;
        open.kf = open.powerAtTest * voltsRatio / testRpm;
        if (!taus.isEmpty()) {
            open.tauS = median(taus);
            open.deadTimeS = median(deadTimes);
        }
        int nearest = 0;
        for (int i = 1; i < n; i++) {
            if (Math.abs(steadyRpm[i] - testRpm) < Math.abs(steadyRpm[nearest] - testRpm)) nearest = i;
        }
        open.noiseRpm = noise[nearest];

        String line = String.format("开环（%.2f V）：每单位功率 %.0f RPM（R² %.4f），%.0f RPM 需要功率 %.3f",
                open.volts, open.rpmPerPower, open.rSquared, testRpm, open.powerAtTest);
        addResult(line);
        addResult(String.format("  换算到 %.1f V：每单位功率 %.0f RPM，KF %s%s", ShooterConstants.NOMINAL_VOLTAGE,
                open.rpmPerPowerNominal, sig(open.kf), volts.count() > 0 ? "" : "（没读到电压，未换算）"));
        addResult(String.format("  时间常数 %s s，滞后 %s s，测速噪声 ±%.1f RPM",
                sig(open.tauS), sig(open.deadTimeS), open.noiseRpm));
        RobotLog.ii(TAG, "%s", line);
        flushCsv();
        return open;
    }

    /**
     * Least-squares grid fit of the normalized step response y = 1 - exp(-(t - dead) / tau). Returns
     * {tau, dead} in seconds, or null when the step is too small to fit.
     */
    static double[] fitFirstOrder(List<double[]> samples, double from, double to) {
        double span = to - from;
        if (Math.abs(span) < 50 || samples.size() < 5) return null;
        double bestError = Double.POSITIVE_INFINITY, bestTau = Double.NaN, bestDead = 0;
        for (int d = 0; d <= 10; d++) {
            double dead = d * 0.01;
            for (double tau = 0.02; tau <= 3.0; tau += 0.005) {
                double error = 0;
                for (double[] s : samples) {
                    double y = (s[1] - from) / span;
                    double model = s[0] <= dead ? 0 : 1 - Math.exp(-(s[0] - dead) / tau);
                    error += (y - model) * (y - model);
                }
                if (error < bestError) {
                    bestError = error;
                    bestTau = tau;
                    bestDead = dead;
                }
            }
        }
        return new double[]{bestTau, bestDead};
    }

    // ---------------------------------------------------------------- closed loop

    private static final class Candidate {
        final String label;
        final double kp, ki, kd, kf;
        double jitter = Double.NaN;           // RPM standard deviation while steady
        double offset = Double.NaN;           // steady mean RPM minus target
        double powerJitter = Double.NaN;
        double recoveryError = Double.NaN;    // mean |error| SCORE_FROM_S-SCORE_TO_S after a dip
        double overshoot = Double.NaN;
        double recoveryS = Double.NaN;
        double score = Double.NaN;

        Candidate(String label, double kp, double ki, double kd, double kf) {
            this.label = label;
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.kf = kf;
        }

        String gains() {
            return String.format("KP %s KI %s KD %s KF %s", sig(kp), sig(ki), sig(kd), sig(kf));
        }

        String metrics() {
            return String.format("误差 %.0f / 抖动 %.0f / 过冲 %.0f / 恢复 %.2f s / 得分 %.0f",
                    recoveryError, jitter, overshoot, recoveryS, score);
        }
    }

    private Candidate testCandidate(String label, int index, int total,
                                    double kp, double ki, double kd, double kf) {
        Candidate c = new Candidate(label, kp, ki, kd, kf);
        String prefix = String.format("%s %d/%d：%s", index <= 1 + LOOP_GAINS.length ? "2/3 KP" : "3/3 KI",
                index, total, c.gains());
        section = label + " | " + c.gains();
        note("candidate " + section);
        shooter.setPowerOverride(Double.NaN);
        shooter.setGains(kp, ki, kd, kf);
        shooter.setTargetRpm(testRpm);
        shooter.startControl(now());
        if (!runClosedLoop(CLOSED_SETTLE_S, "settle", prefix + "，稳定中", null)) return null;

        Stats rpm = new Stats(), power = new Stats();
        if (!runClosedLoop(SAMPLE_S, "steady", prefix + "，测稳态抖动", (r, p) -> {
            rpm.add(r);
            power.add(p);
        })) return null;
        c.jitter = rpm.std();
        c.offset = rpm.mean() - testRpm;
        c.powerJitter = power.std();

        Stats errors = new Stats(), overshoots = new Stats(), recoveries = new Stats();
        for (int dip = 0; dip < DIPS_PER_CANDIDATE; dip++) {
            String status = String.format("%s，掉速测试 %d/%d", prefix, dip + 1, DIPS_PER_CANDIDATE);
            // Brake until DIP_RPM is gone; the controller keeps filtering and integrating meanwhile.
            shooter.setPowerOverride(ShooterConstants.MIN_POWER);
            double dipStart = now();
            double lowest = Double.POSITIVE_INFINITY;
            while (now() - dipStart < MAX_DIP_S) {
                if (!controlTick()) return null;
                lowest = Math.min(lowest, shooter.getLastRawRpm());
                recordControl("dip " + (dip + 1));
                show(status, shooter.getLastRawRpm(), shooter.getPower());
                if (shooter.getLastRawRpm() <= testRpm - DIP_RPM) break;
            }
            shooter.setPowerOverride(Double.NaN);

            double release = now();
            List<double[]> recovery = new ArrayList<>();
            while (now() - release < RECORD_S) {
                if (!controlTick()) return null;
                double r = shooter.getLastRawRpm();
                recovery.add(new double[]{now() - release, r});
                lowest = Math.min(lowest, r);
                recordControl("recover " + (dip + 1));
                show(status, r, shooter.getPower());
            }
            // Depth from the candidate's own steady RPM. Only the dip transient is scaled to DIP_RPM;
            // a steady offset (wrong KF, no KI) counts in full.
            double depth = testRpm + c.offset - lowest;
            if (depth >= DIP_RPM / 3) {
                double scale = DIP_RPM / depth;
                Stats window = new Stats();
                double peak = Double.NEGATIVE_INFINITY;
                double recoveredAt = RECORD_S;
                for (int k = 0; k < recovery.size(); k++) {
                    double t = recovery.get(k)[0];
                    double error = c.offset + (recovery.get(k)[1] - testRpm - c.offset) * scale;
                    if (t >= SCORE_FROM_S && t <= SCORE_TO_S) window.add(Math.abs(error));
                    double smoothed = smoothedError(recovery, k);
                    peak = Math.max(peak, c.offset + (smoothed - c.offset) * scale);
                    if (recoveredAt == RECORD_S && Math.abs(smoothed) <= RECOVERED_RPM) recoveredAt = t;
                }
                errors.add(window.mean());
                overshoots.add(Math.max(0, peak));
                recoveries.add(recoveredAt);
            } else {
                RobotLog.ww(TAG, "%s: dip %d only %.0f RPM deep, skipped", label, dip + 1, depth);
            }
            if (!runClosedLoop(BETWEEN_DIPS_S, "between", status + "，回稳", null)) return null;
        }
        if (errors.count() > 0) {
            c.recoveryError = errors.mean();
            c.overshoot = overshoots.mean();
            c.recoveryS = recoveries.mean();
            c.score = c.recoveryError + c.jitter;
        }
        String line = String.format("%s：%s", label, c.metrics());
        addResult(line);
        String detail = String.format(Locale.US, "%s | %s | offset %.1f RPM, power jitter %.3f",
                c.gains(), c.metrics(), c.offset, c.powerJitter);
        RobotLog.ii(TAG, "%s", detail);
        note(detail);
        flushCsv();
        return c;
    }

    /** Three-sample moving average of the RPM error around sample k. */
    private double smoothedError(List<double[]> samples, int k) {
        double sum = 0;
        int count = 0;
        for (int j = Math.max(0, k - 1); j <= Math.min(samples.size() - 1, k + 1); j++) {
            sum += samples.get(j)[1] - testRpm;
            count++;
        }
        return sum / count;
    }

    /** Lowest score among candidates that do not overshoot; if all overshoot, the least overshoot. */
    private static Candidate pickBest(List<Candidate> candidates) {
        Candidate best = null;
        for (Candidate c : candidates) {
            if (!Double.isFinite(c.score) || c.overshoot > OVERSHOOT_LIMIT_RPM) continue;
            if (best == null || c.score < best.score) best = c;
        }
        if (best != null) return best;
        for (Candidate c : candidates) {
            if (!Double.isFinite(c.overshoot)) continue;
            if (best == null || c.overshoot < best.overshoot) best = c;
        }
        return best != null ? best : candidates.get(0);
    }

    private boolean runClosedLoop(double seconds, String phase, String status,
                                  BiConsumer<Double, Double> sample) {
        double start = now();
        while (now() - start < seconds) {
            if (!controlTick()) return false;
            double rpm = shooter.getLastRawRpm();
            recordControl(phase);
            if (sample != null) sample.accept(rpm, shooter.getPower());
            show(status, rpm, shooter.getPower());
        }
        return true;
    }

    private boolean controlTick() {
        if (!tick()) return false;
        shooter.updateControl(now(), Double.NaN);
        return true;
    }

    // ---------------------------------------------------------------- report

    private List<String> report(OpenLoop open, Candidate baseline, Candidate bestKp, Candidate best,
                                List<Candidate> kpRuns, double startVolts, double endVolts) {
        List<String> lines = new ArrayList<>();
        lines.add("==== 推荐值：复制到 ShooterConstants ====");
        lines.add("KP = " + sig(best.kp) + ";");
        lines.add("KI = " + sig(best.ki) + ";");
        lines.add("KD = " + sig(0) + ";");
        lines.add("KF = " + sig(best.kf) + ";");
        lines.add(String.format("条件：%.0f RPM，循环周期 %.0f ms（实际平均 %.1f ms，最长 %.1f ms），电池 %.2f → %.2f V",
                testRpm, loopMs, averageLoopMs(), loopMsMax, startVolts, endVolts));
        lines.add(String.format("掉 %.0f RPM 后 %.2f–%.2f s 的平均误差 / 稳态抖动 / 过冲 (RPM)：",
                DIP_RPM, SCORE_FROM_S, SCORE_TO_S));
        lines.add("  当前常数（" + baseline.gains() + "）");
        lines.add("    " + baseline.metrics());
        lines.add("  推荐");
        lines.add("    " + best.metrics());
        if (bestKp == kpRuns.get(kpRuns.size() - 1)) {
            lines.add("注意：推荐 KP 是候选里最大的，可以在 LOOP_GAINS 里加更大的值再测。");
        }
        if (!(bestKp.overshoot <= OVERSHOOT_LIMIT_RPM)) {
            lines.add("注意：所有 KP 都过冲超过 " + (int) OVERSHOOT_LIMIT_RPM + " RPM，选的是过冲最小的。");
        }
        if (open.powerAtTest > POWER_HEADROOM_WARNING) {
            lines.add(String.format("注意：%.0f RPM 已经要 %.2f 功率，掉速后能用来恢复的余量很少。",
                    testRpm, open.powerAtTest));
        }
        if (Math.abs(endVolts - startVolts) > 0.5) {
            lines.add("注意：测试中电池电压变化超过 0.5 V，KF 会偏。充满电再测一次更准。");
        }
        lines.add(String.format("KF、KP 已换算到 %.1f V（NOMINAL_VOLTAGE）。Shooter 按电池电压缩放输出，KI 保持 0，"
                + "KI 候选只供参考。", ShooterConstants.NOMINAL_VOLTAGE));
        lines.add(csvFile != null && csv != null ? "数据文件：" + csvFile.getAbsolutePath() : "数据文件没有写成功");
        lines.add("---- 全部结果 ----");
        lines.addAll(resultLines);
        for (String line : lines) {
            RobotLog.ii(TAG, "%s", line);
            note("report " + line);
        }
        flushCsv();
        return lines;
    }

    // ---------------------------------------------------------------- data file

    private void addResult(String line) {
        resultLines.add(line);
        note("result " + line);
    }

    private void openCsv() {
        try {
            File folder = new File(AppUtil.ROOT_FOLDER, CSV_FOLDER);
            if (!folder.isDirectory() && !folder.mkdirs()) throw new IOException("cannot create " + folder);
            String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
            csvFile = new File(folder, "tuner-" + stamp + ".csv");
            csv = new BufferedWriter(new FileWriter(csvFile), 1 << 16);
            note(String.format(Locale.US, "ShooterAutoTuner test_rpm=%.0f loop_ms=%.0f dip_rpm=%.0f battery=%.2f V",
                    testRpm, loopMs, DIP_RPM, readVoltage()));
            note(String.format(Locale.US, "constants KP=%s KI=%s KD=%s KF=%s", sig(ShooterConstants.KP),
                    sig(ShooterConstants.KI), sig(ShooterConstants.KD), sig(ShooterConstants.KF)));
            csv.write(CSV_HEADER);
            csv.newLine();
        } catch (IOException e) {
            RobotLog.ee(TAG, "data file: %s", e.getMessage());
            csv = null;
        }
    }

    private void recordControl(String phase) {
        record(phase, shooter.getPower(), shooter.getLastRawRpm(), shooter.getFilteredRpm(), testRpm);
    }

    private void record(String phase, double power, double rawRpm, double filteredRpm, double targetRpm) {
        if (csv == null) return;
        try {
            csv.write(String.format(Locale.US, "%.4f,%s,%s,%.4f,%.1f,%s,%s", now(), section, phase, power,
                    rawRpm, Double.isNaN(filteredRpm) ? "" : String.format(Locale.US, "%.1f", filteredRpm),
                    Double.isNaN(targetRpm) ? "" : String.format(Locale.US, "%.0f", targetRpm)));
            csv.newLine();
        } catch (IOException e) {
            RobotLog.ee(TAG, "data file: %s", e.getMessage());
            csv = null;
        }
    }

    /** A line starting with # (results, the report, the test conditions). */
    private void note(String line) {
        if (csv == null) return;
        try {
            csv.write("# " + line.replace('\n', ' '));
            csv.newLine();
        } catch (IOException e) {
            RobotLog.ee(TAG, "data file: %s", e.getMessage());
            csv = null;
        }
    }

    private void flushCsv() {
        if (csv == null) return;
        try {
            csv.flush();
        } catch (IOException e) {
            RobotLog.ee(TAG, "data file: %s", e.getMessage());
            csv = null;
        }
    }

    private void closeCsv() {
        if (csv == null) return;
        try {
            csv.close();
        } catch (IOException e) {
            RobotLog.ee(TAG, "data file: %s", e.getMessage());
        }
        csv = null;
    }

    private void holdScreen(List<String> lines) {
        shooter.setPowerOverride(Double.NaN);
        shooter.stop();
        while (opModeIsActive()) {
            for (String line : lines) telemetry.addLine(line);
            telemetry.update();
            sleep(100);
        }
    }

    // ---------------------------------------------------------------- loop timing and telemetry

    /** Waits for the next loop period and records the loop time. False once the OpMode stops. */
    private boolean tick() {
        if (!opModeIsActive()) return false;
        nextLoopNs += Math.round(loopMs * 1e6);
        long nowNs = System.nanoTime();
        if (nextLoopNs < nowNs) nextLoopNs = nowNs;   // behind schedule: do not try to catch up
        while (opModeIsActive() && System.nanoTime() < nextLoopNs) {
            long remainingMs = (nextLoopNs - System.nanoTime()) / 1_000_000;
            if (remainingMs >= 2) sleep(remainingMs - 1);
            else Thread.yield();
        }
        long loopNs = System.nanoTime();
        if (lastLoopNs != 0) {
            double ms = (loopNs - lastLoopNs) / 1e6;
            loopMsSum += ms;
            loopCount++;
            loopMsMax = Math.max(loopMsMax, ms);
        }
        lastLoopNs = loopNs;
        return opModeIsActive();
    }

    private double now() {
        return (System.nanoTime() - startNs) / 1e9;
    }

    private double averageLoopMs() {
        return loopCount > 0 ? loopMsSum / loopCount : Double.NaN;
    }

    private void show(String status, double rpm, double power) {
        double t = now();
        if (t - lastTelemetryS < 0.1) return;
        lastTelemetryS = t;
        telemetry.addLine("飞轮自动调参（按 STOP 随时停止）");
        telemetry.addLine(status);
        telemetry.addData("实测 RPM / 功率", "%.0f / %.3f", rpm, power);
        telemetry.addData("测试转速", "%.0f RPM", testRpm);
        telemetry.addData("循环", "设定 %.0f ms，实际平均 %.1f ms", loopMs, averageLoopMs());
        for (String line : resultLines) telemetry.addLine(line);
        telemetry.update();
    }

    private double readVoltage() {
        double lowest = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltageSensors) {
            double volts = sensor.getVoltage();
            if (volts > 0) lowest = Math.min(lowest, volts);
        }
        return Double.isInfinite(lowest) ? Double.NaN : lowest;
    }

    // ---------------------------------------------------------------- math

    private static final class Stats {
        private double sum;
        private double sumSquares;
        private int count;

        void add(double x) {
            sum += x;
            sumSquares += x * x;
            count++;
        }

        int count() {
            return count;
        }

        double mean() {
            return count > 0 ? sum / count : Double.NaN;
        }

        double std() {
            if (count < 2) return Double.NaN;
            double mean = sum / count;
            return Math.sqrt(Math.max(0, sumSquares / count - mean * mean));
        }
    }

    private static double median(List<Double> values) {
        List<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        int n = sorted.size();
        return n % 2 == 1 ? sorted.get(n / 2) : (sorted.get(n / 2 - 1) + sorted.get(n / 2)) / 2;
    }

    /** Three significant digits as a plain Java double literal, e.g. 0.00134 or 0.0. */
    static String sig(double x) {
        if (Double.isNaN(x)) return "-";
        if (x == 0) return "0.0";
        String s = new BigDecimal(x).round(new MathContext(3)).stripTrailingZeros().toPlainString();
        return s.contains(".") ? s : s + ".0";
    }
}
