package org.firstinspires.ftc.teamcode.components;

import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Constants;

import java.util.Arrays;

/**
 * Turret driven by two continuous servos with different gear ratios.
 *
 * Each servo has an analog position wire. Encoder A turns 6/5 = 1.20 times per turret turn,
 * encoder B turns 5/4 = 1.25 times per turret turn. Their wrapped angles drift apart by
 * 0.05 * turret angle, which gives a coarse multi-turn turret angle (vernier). Encoder B then
 * refines it (finer resolution).
 *
 * Control is a cascade:
 *   shared position PID (turret deg -> turret deg/s)
 *     -> per-servo speed loop (servo deg/s -> power) = feedforward + PID on measured servo speed
 *
 * One position loop is shared on purpose: the two servos are rigidly coupled through the turret,
 * so two separate position loops with different gains would fight each other. Each servo still
 * has its own speed PID, so they share the load.
 *
 * Credit to Claude (dont blame the programmer if shit)
 */
public class Turret {
    public CRServo turreta;
    public CRServo turretb;
    private final AnalogInput anIn_A;
    private final AnalogInput anIn_B;
    private final ComponentShell comps;

    private PIDController posPID;
    private PIDController velPID_A;
    private PIDController velPID_B;
    private double[] lastGains = null;

    private boolean haveLast = false;
    private long lastTimeNs = 0;
    private double lastA, lastB;
    private double velA, velB;                         // measured servo speeds, deg/s (encoder frame)

    private double turretAngle;                        // robot-relative, degrees, multi-turn
    private double targetAngle;
    private double turretVelCmd;
    private double lastTargetGlobal = 0;

    public Turret(HardwareMap hwm, ComponentShell comps) {
        this.turreta = hwm.get(CRServo.class, "turreta");
        this.turretb = hwm.get(CRServo.class, "turretb");
        this.anIn_A = hwm.get(AnalogInput.class, "anIna");
        this.anIn_B = hwm.get(AnalogInput.class, "anInb");

        turretb.setDirection(DcMotorSimple.Direction.REVERSE);
        this.comps = comps;
        updateGains();
    }

    public void update() {
        updateGains();

        // ---- read encoders (degrees, 0..360, sign-corrected so both increase with turret angle) ----
        double a = wrap360(Constants.Turret.ENC_DIR_A * voltsToDeg(anIn_A));
        double b = wrap360(Constants.Turret.ENC_DIR_B * voltsToDeg(anIn_B));

        long now = System.nanoTime();
        double dt = (now - lastTimeNs) * 1e-9;
        if (haveLast && dt > 0.001) {
            velA += Constants.Turret.VEL_FILTER * (wrap180(a - lastA) / dt - velA);
            velB += Constants.Turret.VEL_FILTER * (wrap180(b - lastB) / dt - velB);
        }
        if (!haveLast || dt > 0.001) {
            lastA = a;
            lastB = b;
            lastTimeNs = now;
            haveLast = true;
        }

        turretAngle = turretAngleFromEncoders(a, b) - Constants.Turret.ZERO_OFFSET;

        // ---- where should the turret point? (your shooter math, unchanged) ----
        Pose pose = comps.follower.pose();
        double targetAngleGlobal = Math.atan2(Constants.Field.HIVE_POS.y() - pose.y(), Constants.Field.HIVE_POS.x() - pose.x());

        double deltaX = Math.sqrt((pose.x() - Constants.Field.HIVE_POS.x()) * (pose.x() - Constants.Field.HIVE_POS.x())
                + (pose.y() - Constants.Field.HIVE_POS.y()) * (pose.y() - Constants.Field.HIVE_POS.y()));

        double alpha = Math.toRadians(Constants.Shooter.SHOOTER_ANGLE);
        double targetVTotal = (1 / (2 * Math.sin(alpha))) * Math.sqrt(-2 * Constants.Field.GRAV_ACC * Constants.Field.DELTA_HEIGHT + 2 * Constants.Field.GRAV_ACC * deltaX * Math.tan(alpha)
                + Math.sqrt((2 * Constants.Field.GRAV_ACC * Constants.Field.DELTA_HEIGHT - 2 * Constants.Field.GRAV_ACC * deltaX * Math.tan(alpha))
                * (2 * Constants.Field.GRAV_ACC * Constants.Field.DELTA_HEIGHT - 2 * Constants.Field.GRAV_ACC * deltaX * Math.tan(alpha))
                + 8 * Constants.Field.GRAV_ACC * Constants.Field.GRAV_ACC * deltaX * deltaX * Math.tan(alpha) * Math.tan(alpha)))
                * Constants.Shooter.AIR_RES_COMP_MULT; // Source: See shooter

        double targetV = Math.cos(alpha) * targetVTotal;
        double targetVX = Math.cos(targetAngleGlobal) * targetV;
        double targetVY = Math.sin(targetAngleGlobal) * targetV;

        double vX = targetVX - comps.follower.velocity().vx;
        double vY = targetVY - comps.follower.velocity().vy;

        double targetSOTMAngleGlobal = Math.atan2(vY, vX);
        // if the shot is out of range the sqrt above gives NaN; keep the last good aim instead of sending NaN to the servos
        if (Double.isFinite(targetSOTMAngleGlobal)) {
            lastTargetGlobal = targetSOTMAngleGlobal;
        } else {
            targetSOTMAngleGlobal = lastTargetGlobal;
        }

        // field angle -> robot-relative turret angle (this was missing before), then pick the
        // equivalent angle (+-360k) nearest to where the turret is, inside the soft limits
        double desired = Math.toDegrees(targetSOTMAngleGlobal - pose.heading());
        targetAngle = pickTarget(desired, turretAngle);

        // ---- position loop: turret degrees -> turret deg/s ----
        double error = targetAngle - turretAngle;
        // when the robot spins, the turret must counter-rotate by the same rate
        double omegaFF = -Constants.Turret.OMEGA_FF * Math.toDegrees(comps.follower.velocity().omega);
        double max = Constants.Turret.MAX_TURRET_VEL;
        turretVelCmd = clamp(posPID.calculate(0, error) + omegaFF, -max, max);

        // ---- speed loops: each servo tracks its own share of that speed ----
        turreta.setPower(servoPower(velPID_A, Constants.Turret.RATIO_A * turretVelCmd, velA, Constants.Turret.KV_A, Constants.Turret.KS_A));
        turretb.setPower(servoPower(velPID_B, Constants.Turret.RATIO_B * turretVelCmd, velB, Constants.Turret.KV_B, Constants.Turret.KS_B));
    }

    public void stop() {
        turreta.setPower(0);
        turretb.setPower(0);
    }

    // ---- getters for telemetry ----
    public double angle() { return turretAngle; }
    public double targetAngle() { return targetAngle; }
    public double velCmd() { return turretVelCmd; }
    public double servoVelA() { return velA; }
    public double servoVelB() { return velB; }

    // ---------------------------------------------------------------------------------------

    private double servoPower(PIDController pid, double targetVel, double measuredVel, double kV, double kS) {
        double ff = kV * targetVel + (Math.abs(targetVel) > 2.0 ? Math.copySign(kS, targetVel) : 0.0);
        double fb = pid.calculate(targetVel, targetVel - measuredVel);
        return clamp(ff + fb, -1.0, 1.0);
    }

    /**
     * a, b are encoder angles in degrees (0..360). Returns the multi-turn turret angle in degrees.
     *   a = 1.20 * theta (mod 360),  b = 1.25 * theta (mod 360)
     *   b - a = 0.05 * theta (mod 360)  -> coarse theta = 20 * (b - a), centred on 0
     *   then use b to refine: theta = (b + 360 m) / 1.25 for the integer m closest to the coarse value
     */
    private double turretAngleFromEncoders(double a, double b) {
        double d = wrap180(b - a);
        double coarse = Constants.Turret.VERNIER_GAIN * d;
        double m = Math.round((Constants.Turret.RATIO_B * coarse - b) / 360.0);
        return (b + 360.0 * m) / Constants.Turret.RATIO_B;
    }

    private double pickTarget(double desired, double current) {
        double lo = Constants.Turret.MIN_ANGLE;
        double hi = Constants.Turret.MAX_ANGLE;
        double t = desired + 360.0 * Math.round((current - desired) / 360.0);   // equivalent angle nearest the turret
        if (t > hi) t -= 360.0;
        if (t < lo) t += 360.0;
        return clamp(t, lo, hi);
    }

    private void updateGains() {
        double[] now = {
                Constants.Turret.POS_P, Constants.Turret.POS_D,
                Constants.Turret.VEL_P_A, Constants.Turret.VEL_D_A,
                Constants.Turret.VEL_P_B, Constants.Turret.VEL_D_B
        };
        if (lastGains == null || !Arrays.equals(now, lastGains)) {
            lastGains = now;
            posPID = Controller.pid(now[0], 0, now[1]);
            velPID_A = Controller.pid(now[2], 0, now[3]);
            velPID_B = Controller.pid(now[4], 0, now[5]);
        }
    }

    private static double voltsToDeg(AnalogInput in) {
        return in.getVoltage() / in.getMaxVoltage() * 360.0;
    }

    private static double wrap360(double deg) {
        return ((deg % 360.0) + 360.0) % 360.0;
    }

    private static double wrap180(double deg) {
        return wrap360(deg + 180.0) - 180.0;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}