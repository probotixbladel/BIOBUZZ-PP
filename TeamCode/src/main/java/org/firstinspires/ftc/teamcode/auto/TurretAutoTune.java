package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * Turret auto tuner. Run it with the turret roughly in the middle of its range and about
 * 150 degrees of free travel both ways. Press STOP at any time to abort.
 *
 * Phase 1: direction checks (are both servos and both encoders consistent?)
 * Phase 2: feedforward (KS, KV) from steady-state speed at several powers
 * Phase 3: step response -> time constant -> speed-loop P and position-loop P
 *
 * It only measures SPEED dynamics. It does not set MIN_ANGLE / MAX_ANGLE / ZERO_OFFSET /
 * OMEGA_FF, and it does not replace a quick sanity check on the real robot afterwards.
 */
@Autonomous(name = "TurretAutoTune", group = "Tuning")
public class TurretAutoTune extends LinearOpMode {
    // ---- keep these identical to Turret.java ----
    private static final String RIGHT_SERVO = "turretRight";
    private static final String LEFT_SERVO = "turretLeft";
    private static final String RIGHT_ENC = "anInRight";
    private static final String LEFT_ENC = "anInLeft";

    // ---- test settings ----
    private static final double SAFE_POWER = 0.25;                       // direction checks
    private static final double[] LEVELS = {0.10, 0.15, 0.20, 0.30, 0.40};
    private static final double PULSE_SEC = 0.7;
    private static final double SETTLE_SEC = 0.3;                        // ignored part of each pulse
    private static final double STEP_SEC = 1.0;
    private static final double MAX_TRAVEL = 150;                        // deg of turret travel from the start, abort beyond this
    private static final double MAX_SPEED = 220;                         // stop raising power above this turret speed (deg/s)
    private static final double MIN_MOVE = 10;                           // below this speed we say "did not move" (deg/s)
    private static final double LOOP_DELAY = 0.06;                       // rough delay of the real control loop incl. speed filter (s)
    private static final double WINDOW = 0.10;                           // window for speed estimates in the step response (s)

    private CRServo right, left;
    private AnalogInput anRight, anLeft;

    private double travel = 0;       // net turret travel since the start (deg)
    private boolean aborted = false;

    private double kvComb;           // power per turret deg/s, equal power on both servos
    private double ksComb;           // power needed to move
    private double stepPower = 0.2;
    private double tau;

    private static class Trace {
        final List<Double> t = new ArrayList<>();
        final List<Double> th = new ArrayList<>();    // turret angle estimate (average of both encoders)
        final List<Double> thR = new ArrayList<>();   // turret angle estimate from the right encoder
        final List<Double> thL = new ArrayList<>();   // turret angle estimate from the left encoder
    }

    @Override
    public void runOpMode() {
        right = hardwareMap.get(CRServo.class, RIGHT_SERVO);
        left = hardwareMap.get(CRServo.class, LEFT_SERVO);
        left.setDirection(DcMotorSimple.Direction.REVERSE);          // same as Turret.java
        right.setDirection(DcMotorSimple.Direction.REVERSE);
        anRight = hardwareMap.get(AnalogInput.class, RIGHT_ENC);
        anLeft = hardwareMap.get(AnalogInput.class, LEFT_ENC);

        telemetry.setAutoClear(false);
        say("Turret mid-range, ~150 deg free each way. START to run, STOP aborts.");
        waitForStart();
        if (!opModeIsActive()) return;

        if (checkDirections() && tuneFeedforward() && tuneDynamics()) {
            computeAndApply();
        } else {
            say("TUNING STOPPED. Fix the problem above and run again.");
        }
        stopServos();
        while (opModeIsActive()) sleep(100);
    }

    // ------------------------------------------------------------------ phases

    private boolean checkDirections() {
        say("--- Phase 1: directions ---");
        Trace both = pulse(SAFE_POWER, SAFE_POWER, 0.5);
        pulse(-SAFE_POWER, -SAFE_POWER, 0.5);
        settle();
        if (aborted) return false;

        double sR = slope(both.t, both.thR, 0.15, 0.5);
        double sL = slope(both.t, both.thL, 0.15, 0.5);
        say(String.format("both servos +%.2f: right enc %.0f, left enc %.0f (turret deg/s)", SAFE_POWER, sR, sL));

        if (Math.abs(sR) < MIN_MOVE && Math.abs(sL) < MIN_MOVE) {
            say("FAIL: the turret did not move. Check power, wiring and that it is free to turn.");
            return false;
        }
        if (sR * sL < 0) {
            say("FAIL: the two encoders move in OPPOSITE directions. Flip ENC_DIR_RIGHT or ENC_DIR_LEFT (only one of them).");
            return false;
        }
        if (sR < 0 && sL < 0) {
            say("FAIL: positive power makes the angle go DOWN. Reverse BOTH servos (right REVERSE, left FORWARD) and run again.");
            return false;
        }
        double rel = sL / sR;
        if (rel < 0.85 || rel > 1.15) {
            say(String.format("WARN: encoders disagree by %.0f%%. Gear ratios may be wrong or something slips.", Math.abs(rel - 1) * 100));
        }

        boolean ok = true;
        for (int i = 0; i < 2; i++) {
            String name = i == 0 ? "RIGHT" : "LEFT";
            double pr = i == 0 ? SAFE_POWER : 0;
            double pl = i == 1 ? SAFE_POWER : 0;
            Trace one = pulse(pr, pl, 0.5);
            pulse(-pr, -pl, 0.5);
            settle();
            if (aborted) return false;
            double s = slope(one.t, one.th, 0.15, 0.5);
            if (s < -MIN_MOVE) {
                say(String.format("FAIL: %s servo alone pushes the turret the WRONG way (%.0f deg/s). Toggle its setDirection.", name, s));
                ok = false;
            } else if (s < MIN_MOVE) {
                say(String.format("WARN: %s servo alone barely moves the turret (%.0f deg/s). Weak or slipping?", name, s));
            } else {
                say(String.format("OK: %s servo alone pushes the right way (%.0f deg/s)", name, s));
            }
        }
        return ok;
    }

    private boolean tuneFeedforward() {
        say("--- Phase 2: feedforward ---");
        List<double[]> pts = new ArrayList<>();
        double bestDist = Double.MAX_VALUE;
        for (double p : LEVELS) {
            Trace up = pulse(p, p, PULSE_SEC);
            settle();
            Trace dn = pulse(-p, -p, PULSE_SEC);
            settle();
            if (aborted) return false;
            double wUp = slope(up.t, up.th, SETTLE_SEC, PULSE_SEC);
            double wDn = slope(dn.t, dn.th, SETTLE_SEC, PULSE_SEC);
            double w = 0.5 * (Math.abs(wUp) + Math.abs(wDn));
            say(String.format("power %.2f -> %.0f deg/s (up %.0f, down %.0f)", p, w, wUp, wDn));
            if (w > MIN_MOVE) {
                pts.add(new double[]{w, p});
                if (Math.abs(w - 100) < bestDist) {
                    bestDist = Math.abs(w - 100);
                    stepPower = p;
                }
            }
            if (w > MAX_SPEED) break;
        }
        if (pts.size() < 3) {
            say("FAIL: fewer than 3 usable points. Servos too weak, or too much friction, for these power levels.");
            return false;
        }
        // least squares: p = ks + kv * w
        int n = pts.size();
        double sw = 0, sp = 0, sww = 0, swp = 0;
        for (double[] pt : pts) {
            sw += pt[0];
            sp += pt[1];
            sww += pt[0] * pt[0];
            swp += pt[0] * pt[1];
        }
        double den = n * sww - sw * sw;
        kvComb = (n * swp - sw * sp) / den;
        ksComb = Math.max(0, (sp - kvComb * sw) / n);
        if (kvComb <= 0) {
            say("FAIL: power did not increase speed (kv <= 0). Check the test data above.");
            return false;
        }
        say(String.format("fit: KV = %.5f power per deg/s, KS = %.3f", kvComb, ksComb));
        return true;
    }

    private boolean tuneDynamics() {
        say("--- Phase 3: step response ---");
        double sum = 0;
        int cnt = 0;
        for (int dir = 1; dir >= -1; dir -= 2) {
            Trace tr = pulse(dir * stepPower, dir * stepPower, STEP_SEC);
            settle();
            if (aborted) return false;
            double wss = dir * slope(tr.t, tr.th, 0.6 * STEP_SEC, STEP_SEC);
            if (wss < MIN_MOVE) continue;
            double target = 0.632 * wss;
            for (double c = WINDOW / 2; c <= STEP_SEC - WINDOW / 2; c += 0.01) {
                double s = dir * slope(tr.t, tr.th, c - WINDOW / 2, c + WINDOW / 2);
                if (s >= target) {
                    sum += c;
                    cnt++;
                    say(String.format("step %s: final %.0f deg/s, reached 63%% at %.3f s", dir > 0 ? "up" : "down", wss, c));
                    break;
                }
            }
        }
        if (cnt == 0) {
            say("FAIL: could not find a time constant (speed never reached 63% inside the window).");
            return false;
        }
        tau = Math.max(0.03, Math.min(1.0, sum / cnt));
        say(String.format("time constant tau = %.3f s", tau));
        return true;
    }

    private void computeAndApply() {
        double ratioR = Constants.Turret.RATIO_RIGHT;
        double ratioL = Constants.Turret.RATIO_LEFT;
        double avgRatio = 0.5 * (ratioR + ratioL);

        // speed loop: choose loop gain L (closed-loop speed-up = 1 + L), limited by the control loop delay
        double loopGain = Math.max(0.3, Math.min(2.0, 0.8 * tau / LOOP_DELAY));
        double velP = loopGain * kvComb / avgRatio;
        double lambda = tau / (1 + loopGain);                       // closed-loop time constant of the speed loop
        double posP = Math.min(8.0, 1.0 / (4.0 * lambda));          // outer loop ~4x slower than the inner one

        Constants.Turret.KV_RIGHT = kvComb / ratioR;
        Constants.Turret.KV_LEFT = kvComb / ratioL;
        Constants.Turret.KS_RIGHT = ksComb;
        Constants.Turret.KS_LEFT = ksComb;
        Constants.Turret.VEL_P_RIGHT = velP;
        Constants.Turret.VEL_P_LEFT = velP;
        Constants.Turret.VEL_D_RIGHT = 0;
        Constants.Turret.VEL_D_LEFT = 0;
        Constants.Turret.POS_P = posP;
        Constants.Turret.POS_D = 0;

        double maxVel = 0.7 * (1.0 - ksComb) / kvComb;

        say("=== RESULT (paste into Constants.Turret) ===");
        say(String.format("KV_RIGHT = %.5f;  KV_LEFT = %.5f;", Constants.Turret.KV_RIGHT, Constants.Turret.KV_LEFT));
        say(String.format("KS_RIGHT = %.3f;  KS_LEFT = %.3f;", ksComb, ksComb));
        say(String.format("VEL_P_RIGHT = %.5f;  VEL_P_LEFT = %.5f;  VEL_D_* = 0;", velP, velP));
        say(String.format("POS_P = %.2f;  POS_D = 0;", posP));
        say(String.format("MAX_TURRET_VEL suggestion: about %.0f deg/s", maxVel));
        say("Values are also applied in memory until the app restarts. Paste them to keep them.");
    }

    // ------------------------------------------------------------------ helpers

    /** Drives both servos for `sec` seconds and records the turret angle estimate over time. */
    private Trace pulse(double pRight, double pLeft, double sec) {
        Trace tr = new Trace();
        if (aborted) return tr;
        double a = readRight();
        double b = readLeft();
        double unR = 0, unL = 0;
        ElapsedTime clock = new ElapsedTime();
        right.setPower(pRight);
        left.setPower(pLeft);
        double lastTh = 0;
        while (opModeIsActive() && clock.seconds() < sec) {
            double a2 = readRight();
            double b2 = readLeft();
            unR += wrap180(a2 - a);
            unL += wrap180(b2 - b);
            a = a2;
            b = b2;
            double thR = unR / Constants.Turret.RATIO_RIGHT;
            double thL = unL / Constants.Turret.RATIO_LEFT;
            double th = 0.5 * (thR + thL);
            tr.t.add(clock.seconds());
            tr.thR.add(thR);
            tr.thL.add(thL);
            tr.th.add(th);
            lastTh = th;
            if (Math.abs(travel + th) > MAX_TRAVEL) {
                aborted = true;
                say("ABORT: turret moved more than " + MAX_TRAVEL + " deg from where it started.");
                break;
            }
            sleep(3);
        }
        stopServos();
        travel += lastTh;
        return tr;
    }

    private void stopServos() {
        right.setPower(0);
        left.setPower(0);
    }

    private void settle() {
        sleep(300);
    }

    private double readRight() {
        return wrap360(Constants.Turret.ENC_DIR_RIGHT * anRight.getVoltage() / anRight.getMaxVoltage() * 360.0);
    }

    private double readLeft() {
        return wrap360(Constants.Turret.ENC_DIR_LEFT * anLeft.getVoltage() / anLeft.getMaxVoltage() * 360.0);
    }

    /** Least-squares slope of y against t, using samples with t0 <= t <= t1. */
    private static double slope(List<Double> t, List<Double> y, double t0, double t1) {
        int n = 0;
        double st = 0, sy = 0, stt = 0, sty = 0;
        for (int i = 0; i < t.size(); i++) {
            double ti = t.get(i);
            if (ti < t0 || ti > t1) continue;
            double yi = y.get(i);
            n++;
            st += ti;
            sy += yi;
            stt += ti * ti;
            sty += ti * yi;
        }
        double den = n * stt - st * st;
        if (n < 3 || Math.abs(den) < 1e-9) return 0;
        return (n * sty - st * sy) / den;
    }

    private void say(String s) {
        telemetry.addLine(s);
        telemetry.update();
    }

    private static double wrap360(double deg) {
        return ((deg % 360.0) + 360.0) % 360.0;
    }

    private static double wrap180(double deg) {
        return wrap360(deg + 180.0) - 180.0;
    }
}