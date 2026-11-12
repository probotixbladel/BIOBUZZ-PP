package org.firstinspires.ftc.teamcode.components;

import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Turret {
    public CRServo turreta; //Encoder A turns 6/5 = 1.20 times per wheel turn
    public CRServo turretb; //Encoder B turns 5/4 = 1.25 times per wheel turn
    private AnalogInput anIn_A;
    private AnalogInput anIn_B;
    private ComponentShell comps;
    private PIDController pid_A;
    private PIDController pid_B;
    private double kp_A;
    private double kd_A;
    private double kp_B;
    private double kd_B;
    public Turret(HardwareMap hwm, ComponentShell comps) {
        this.turreta = hwm.get(CRServo.class, "turreta");
        this.turretb = hwm.get(CRServo.class, "turretb");
        this.anIn_A = hwm.get(AnalogInput.class, "anIna");
        this.anIn_B = hwm.get(AnalogInput.class, "anInb");

        turretb.setDirection(DcMotorSimple.Direction.REVERSE);
        this.comps = comps;
    }

    public void update() {
        if(Constants.Turret.P_A != kp_A || Constants.Turret.D_A != kd_A) {
            kp_A = Constants.Turret.P_A;
            kd_A = Constants.Turret.D_A;
            pid_A = Controller.pid(kp_A, 0, kd_A);
        }
        if(Constants.Turret.P_B != kp_B || Constants.Turret.D_B != kd_B) {
            kp_B = Constants.Turret.P_B;
            kd_B = Constants.Turret.D_B;
            pid_B = Controller.pid(kp_B, 0, kd_B);
        }

        double targetAngleGlobal = Math.atan2(Constants.Field.HIVE_POS.y() - comps.follower.pose().y(), Constants.Field.HIVE_POS.x() - comps.follower.pose().x());

        double deltaX = Math.sqrt((comps.follower.pose().x() - Constants.Field.HIVE_POS.x()) * (comps.follower.pose().x() - Constants.Field.HIVE_POS.x())
                + (comps.follower.pose().y() - Constants.Field.HIVE_POS.y()) * (comps.follower.pose().y() - Constants.Field.HIVE_POS.y()));

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

        double angle_A = anIn_A.getVoltage() / anIn_A.getMaxVoltage() * 360;
        double angle_B = anIn_B.getVoltage() / anIn_B.getMaxVoltage() * 360;
        double[] pos =  getTurretPos(angle_A, angle_B);

        turreta.setPower(pid_A.calculate(targetSOTMAngleGlobal, targetSOTMAngleGlobal - pos[1]));
        turretb.setPower(pid_B.calculate(targetSOTMAngleGlobal, targetSOTMAngleGlobal - pos[1]));
    }

    private double[] getTurretPos(double a, double b) {
        double d = (a - b) % 360;
        double alphaCoarse = 20 * d;
        double m = Math.round((1.25 * alphaCoarse - b) / 360);
        double alpha = (b + 360 * m) / 1.25;
        return new double[]{(alpha / 360), alpha % 360}; // (turns, angle)
    }   //Encoder A turns 6/5 = 1.20 times per wheel turn
        //Encoder B turns 5/4 = 1.25 times per wheel turn
}
