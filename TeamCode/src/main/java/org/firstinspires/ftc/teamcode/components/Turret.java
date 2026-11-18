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


    public Turret(HardwareMap hwm, ComponentShell comps) {
        // the strings are the device names in the Robot Configuration: left unchanged on purpose
        this.turretRight = hwm.get(CRServo.class, "turretRight");
        this.turretLeft = hwm.get(CRServo.class, "turretLeft");
        this.anIn_Right = hwm.get(AnalogInput.class, "anInRight");
        this.anIn_Left = hwm.get(AnalogInput.class, "anInLeft");

        turretLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.comps = comps;
    }

    public void update() {
        comps.telemetryM.addData("turretAngle: ", wheelPosition(anIn_Right, anIn_Left));
    }

    private double wheelPosition(AnalogInput rightIn, AnalogInput leftIn){
        double right = wrap360(Constants.Turret.ENC_DIR_RIGHT * rightIn.getVoltage() / rightIn.getMaxVoltage() * 360.0);
        double left  = wrap360(Constants.Turret.ENC_DIR_LEFT * leftIn.getVoltage() / leftIn.getMaxVoltage() * 360.0);
        double D = wrap180(left - right);
        double alphaCoarse = 1 / Constants.Turret.RATIO_LEFT * D;
        double m = Math.round((Constants.Turret.RATIO_LEFT * alphaCoarse - left) / 360);
        double alpha = (left + 360 * m) / Constants.Turret.RATIO_LEFT;
        return alpha + Math.toDegrees(comps.follower.pose().heading()) + Constants.Turret.ZERO_OFFSET; // TODO: l + robot + min + origin = norm
    }

    private double wrap180(double angle){
        double a = angle % 360;
        if (a > 180) {
            return -a + 180;
        }
        if (a < -180) {
            return -a - 180;
        }
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