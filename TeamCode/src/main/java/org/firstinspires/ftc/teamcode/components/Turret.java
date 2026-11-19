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
public class Turret {
    public CRServo turretRight;
    public CRServo turretLeft;
    private final AnalogInput anIn_Right;
    private final AnalogInput anIn_Left;
    private final ComponentShell comps;
    private double rightOffset = 0;
    private double leftOffset = 0;
    private Controller pid;
    private double kp;
    private double kd;
    private double kf;


    public Turret(HardwareMap hwm, ComponentShell comps) {
        // the strings are the device names in the Robot Configuration: left unchanged on purpose
        this.turretRight = hwm.get(CRServo.class, "turretRight");
        this.turretLeft = hwm.get(CRServo.class, "turretLeft");
        this.anIn_Right = hwm.get(AnalogInput.class, "anInRight");
        this.anIn_Left = hwm.get(AnalogInput.class, "anInLeft");

        turretLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        turretRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.comps = comps;

        rightOffset = wrap360(Constants.Turret.ENC_DIR_RIGHT * anIn_Right.getVoltage() / anIn_Right.getMaxVoltage() * 360.0);
        leftOffset  = wrap360(Constants.Turret.ENC_DIR_LEFT * anIn_Left.getVoltage() / anIn_Left.getMaxVoltage() * 360.0);
        pid = Controller.pid(kp, 0, kd).plus(Controller.proportionalFeedforward(kf));
    }

    public void update() {

        if(kp != Constants.Turret.P || kd != Constants.Turret.D || kf != Constants.Turret.F) {
            kp = Constants.Turret.P;
            kd = Constants.Turret.D;
            kf = Constants.Turret.F;
            pid = Controller.pid(kp, 0, kd).plus(Controller.proportionalFeedforward(kf));
        }
        comps.telemetryM.addData("turretAngle: ", wheelPosition(anIn_Right, anIn_Left));

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
        double currentAngle = wrap360(wheelPosition(anIn_Right, anIn_Left) - Constants.Turret.MIN_ANGLE);
        double targetAngle = wrap360(wrap360(Math.toDegrees(Math.atan2(hivePos.y() - pose.y(), hivePos.x() - pose.x())) - Math.toDegrees(pose.heading())) - Constants.Turret.MIN_ANGLE);
        comps.telemetryM.addData("TargetRel: ", wrap360(Math.toDegrees(Math.atan2(hivePos.y() - pose.y(), hivePos.x() - pose.x())) - Math.toDegrees(pose.heading())));
        double maxNorm = wrap360(wrap360(Constants.Turret.MAX_ANGLE) - Constants.Turret.MIN_ANGLE);
        comps.telemetryM.addData("can turret: ", targetAngle < maxNorm);

        double target = Math.min(maxNorm, targetAngle);
        if (targetAngle < maxNorm) {
            double maxRatio = Math.max(Constants.Turret.RATIO_RIGHT, Constants.Turret.RATIO_LEFT);
            double rightScale = Constants.Turret.RATIO_RIGHT / maxRatio;   // 1.0
            double leftScale = Constants.Turret.RATIO_LEFT / maxRatio;    // 0.96
            double calc = pid.calculate(targetAngle, targetAngle - currentAngle);
            turretRight.setPower(calc * rightScale);
            turretLeft.setPower(calc * leftScale);
        }
    }

    private double wheelPosition(AnalogInput rightIn, AnalogInput leftIn){
        /*double right = wrap360(Constants.Turret.ENC_DIR_RIGHT * rightIn.getVoltage() / rightIn.getMaxVoltage() * 360.0 - rightOffset);
        double left  = wrap360(Constants.Turret.ENC_DIR_LEFT * leftIn.getVoltage() / leftIn.getMaxVoltage() * 360.0 - leftOffset);
        double D = wrap180(left - right);
        double alphaCoarse = 1 / (Constants.Turret.RATIO_LEFT - Constants.Turret.RATIO_RIGHT) * D;
        double m = Math.round((Constants.Turret.RATIO_LEFT * alphaCoarse - left) / 360);
        double alpha = (left + 360 * m) / Constants.Turret.RATIO_LEFT;
        return alpha;// + Math.toDegrees(comps.follower.pose().heading()) + Constants.Turret.ZERO_OFFSET; // TODO: l + robot + min + origin = */

        // fuck you im just doing the left encoder
        double left = wrap180(Constants.Turret.ENC_DIR_LEFT * leftIn.getVoltage() / leftIn.getMaxVoltage() * 360.0 - leftOffset);
        return left / Constants.Turret.RATIO_LEFT + Constants.Turret.ZERO_OFFSET;   // turret degrees, robot-relative
    }

    private double wrap180(double angle) {
        double a = angle % 360;
        if (a > 180) return a - 360;
        if (a < -180) return a + 360;
        return a;
    }

    private double wrap360(double angle) {
        double a = wrap180(angle);
        if (a < 0) {
            return a + 360;
        }
        return a;
    }
}