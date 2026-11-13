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
 * Each servo has an analog position wire. The right encoder turns 6/5 = 1.20 times per turret turn,
 * the left encoder turns 5/4 = 1.25 times per turret turn. Their wrapped angles drift apart by
 * 0.05 * turret angle, which gives a coarse multi-turn turret angle (vernier). The left encoder then
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
    public CRServo turretRight;
    public CRServo turretLeft;
    private final AnalogInput anIn_Right;
    private final AnalogInput anIn_Left;
    private final ComponentShell comps;

    private PIDController posPID;
    private PIDController velPID_Right;
    private PIDController velPID_Left;
    private double[] lastGains = null;

    private boolean haveLast = false;
    private long lastTimeNs = 0;
    private double lastRight, lastLeft;
    private double velRight, velLeft;                  // measured servo speeds, deg/s (encoder frame)

    private double turretAngle;                        // robot-relative, degrees, multi-turn
    private double targetAngle;
    private double turretVelCmd;
    private double lastTargetGlobal = 0;

    public Turret(HardwareMap hwm, ComponentShell comps) {
        // the strings are the device names in the Robot Configuration: left unchanged on purpose
        this.turretRight = hwm.get(CRServo.class, "turreta");
        this.turretLeft = hwm.get(CRServo.class, "turretb");
        this.anIn_Right = hwm.get(AnalogInput.class, "anIna");
        this.anIn_Left = hwm.get(AnalogInput.class, "anInb");

        turretLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.comps = comps;
        updateGains();
    }

    public void update() {
        updateGains();

        // ---- read encoders (degrees, 0..360, sign-corrected so both increase with turret angle) ----
        double right = wrap360(Constants.Turret.ENC_DIR_RIGHT * voltsToDeg(anIn_Right));
        double left = wrap360(Constants.Turret.ENC_DIR_LEFT * voltsToDeg(anIn_Left));

        long now = System.nanoTime();
        double dt = (now - lastTimeNs) * 1e-9;
        if (haveLast && dt > 0.001) {
            velRight += Constants.Turret.VEL_FILTER * (wrap180(right - lastRight) / dt - velRight);
            velLeft += Constants.Turret.VEL_FILTER * (wrap180(left - lastLeft) / dt - velLeft);
        }
        if (!haveLast || dt > 0.001) {
            lastRight = right;
            lastLeft = left;
            lastTimeNs = now;
            haveLast = true;
        }

        turretAngle = turretAngleFromEncoders(right, left) - Constants.Turret.ZERO_OFFSET;

        // ---- where should the turret point? (your shooter math, unchanged) ---- // sassy claude
        Pose pose = comps.follower.pose();
        Pose hivePos = new Pose(0, 0);
        if (pose.y() > 72) {
            if (comps.alliance == ComponentShell.Alliance.BLUE) {
                hivePos = Constants.Field.HIVE_POS_BLUE_UP;
            }
            else {
                hivePos = Constants.Field.HIVE_POS_RED_UP;
            }
        }
        else {
            if (comps.alliance == ComponentShell.Alliance.BLUE) {
                hivePos = Constants.Field.HIVE_POS_BLUE_DOWN;
            }
            else {
                hivePos = Constants.Field.HIVE_POS_RED_DOWN;
            }
        }
        double targetAngleGlobal = Math.atan2(hivePos.y() - pose.y(), hivePos.x() - pose.x());

        double deltaX = Math.sqrt((pose.x() - hivePos.x()) * (pose.x() - hivePos.x())
                + (pose.y() - hivePos.y()) * (pose.y() - hivePos.y())) * 0.0254;

        double alpha = Math.toRadians(Constants.Shooter.SHOOTER_ANGLE);
        double targetVTotal = Math.sqrt( (-Constants.Field.GRAV_ACC * deltaX * deltaX) / (2 * Math.cos(Math.toRadians(Constants.Shooter.SHOOTER_ANGLE)) * Math.cos(Math.toRadians(Constants.Shooter.SHOOTER_ANGLE))
                * (Math.tan(Math.toRadians(Constants.Shooter.SHOOTER_ANGLE)) * deltaX - Constants.Field.DELTA_HEIGHT))) * Constants.Shooter.AIR_RES_COMP_MULT; // Source: See shooter

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
        turretRight.setPower(servoPower(velPID_Right, Constants.Turret.RATIO_RIGHT * turretVelCmd, velRight, Constants.Turret.KV_RIGHT, Constants.Turret.KS_RIGHT));
        turretLeft.setPower(servoPower(velPID_Left, Constants.Turret.RATIO_LEFT * turretVelCmd, velLeft, Constants.Turret.KV_LEFT, Constants.Turret.KS_LEFT));
    }

    public void stop() {
        turretRight.setPower(0);
        turretLeft.setPower(0);
    }

    // ---- getters for telemetry ----
    public double angle() { return turretAngle; }
    public double targetAngle() { return targetAngle; }
    public double velCmd() { return turretVelCmd; }
    public double servoVelRight() { return velRight; }
    public double servoVelLeft() { return velLeft; }

    // ---------------------------------------------------------------------------------------

    private double servoPower(PIDController pid, double targetVel, double measuredVel, double kV, double kS) {
        double ff = kV * targetVel + (Math.abs(targetVel) > 2.0 ? Math.copySign(kS, targetVel) : 0.0);
        double fb = pid.calculate(targetVel, targetVel - measuredVel);
        return clamp(ff + fb, -1.0, 1.0);
    }

    /**
     * right, left are encoder angles in degrees (0..360). Returns the multi-turn turret angle in degrees.
     *   right = 1.20 * theta (mod 360),  left = 1.25 * theta (mod 360)
     *   left - right = 0.05 * theta (mod 360)  -> coarse theta = 20 * (left - right), centred on 0
     *   then use left to refine: theta = (left + 360 m) / 1.25 for the integer m closest to the coarse value
     */
    private double turretAngleFromEncoders(double right, double left) {
        double d = wrap180(left - right);
        double coarse = Constants.Turret.VERNIER_GAIN * d;
        double m = Math.round((Constants.Turret.RATIO_LEFT * coarse - left) / 360.0);
        return (left + 360.0 * m) / Constants.Turret.RATIO_LEFT;
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
                Constants.Turret.VEL_P_RIGHT, Constants.Turret.VEL_D_RIGHT,
                Constants.Turret.VEL_P_LEFT, Constants.Turret.VEL_D_LEFT
        };
        if (lastGains == null || !Arrays.equals(now, lastGains)) {
            lastGains = now;
            posPID = Controller.pid(now[0], 0, now[1]);
            velPID_Right = Controller.pid(now[2], 0, now[3]);
            velPID_Left = Controller.pid(now[4], 0, now[5]);
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