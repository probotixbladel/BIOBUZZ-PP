package org.firstinspires.ftc.teamcode.components;

import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Shooter {
    private HardwareMap hardwareMap;
    public DcMotorEx shooter;
    private ComponentShell comps;
    private PIDController pid;
    private double kp;
    private double kd;
    private double kf;
    public Shooter(HardwareMap hwm, ComponentShell comps){
        this.hardwareMap = hwm;
        this.shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        this.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.comps = comps;
    }

    public void update() {
        if(kp != Constants.Shooter.P || kd != Constants.Shooter.D) {
            kp = Constants.Shooter.P;
            kd = Constants.Shooter.D;
            pid = Controller.pid(kp, 0, kd);
        }

        double deltaX = Math.sqrt((comps.follower.pose().x() - Constants.Field.HIVE_POS.x()) * (comps.follower.pose().x() - Constants.Field.HIVE_POS.x())
                                + (comps.follower.pose().y() - Constants.Field.HIVE_POS.y()) * (comps.follower.pose().y() - Constants.Field.HIVE_POS.y()));

        double alpha = Math.toRadians(Constants.Shooter.SHOOTER_ANGLE);
        double targetV = (1 / (2 * Math.sin(alpha))) * Math.sqrt(-2 * Constants.Field.GRAV_ACC * Constants.Field.DELTA_HEIGHT + 2 * Constants.Field.GRAV_ACC * deltaX * Math.tan(alpha)
                + Math.sqrt((2 * Constants.Field.GRAV_ACC * Constants.Field.DELTA_HEIGHT - 2 * Constants.Field.GRAV_ACC * deltaX * Math.tan(alpha))
            * (2 * Constants.Field.GRAV_ACC * Constants.Field.DELTA_HEIGHT - 2 * Constants.Field.GRAV_ACC * deltaX * Math.tan(alpha))
                + 8 * Constants.Field.GRAV_ACC * Constants.Field.GRAV_ACC * deltaX * deltaX * Math.tan(alpha) * Math.tan(alpha)))
                * Constants.Shooter.AIR_RES_COMP_MULT; // Source: Just trust me bro


        double targetAngle = Math.atan2(Constants.Field.HIVE_POS.y() - comps.follower.pose().y(), Constants.Field.HIVE_POS.x() - comps.follower.pose().x());
        double targetVX = Math.cos(alpha) * targetV;
        double targetVY = Math.sin(alpha) * targetV;
        double targetVXX = Math.cos(targetAngle) * targetVX;
        double targetVXY = Math.sin(targetAngle) * targetVX;

        double vXX = targetVXX - comps.follower.velocity().vx;
        double vXY = targetVXY - comps.follower.velocity().vy;

        double v = Math.sqrt(vXX * vXX + vXY * vXY + targetVY * targetVY);

        shooter.setVelocity(pid.calculate(toRpm(v), toRpm(v) - shooter.getVelocity()));

    }

    private double toRpm(double v){
        return (v / ( 2 * Math.PI * Constants.Shooter.FLYWHEEL_RADIUS)) * 60;
    }

}
