package org.firstinspires.ftc.teamcode.components;

import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Shooter {
    private HardwareMap hardwareMap;
    public DcMotorEx shooter;
    private ComponentShell comps;
    private Controller pid;
    private double kp;
    private double kd;
    private double kf;
    public double overrideOffset = 0;
    public Shooter(HardwareMap hwm, ComponentShell comps){
        this.hardwareMap = hwm;
        this.shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        this.shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.comps = comps;
    }

    public void update() {
        if(kp != Constants.Shooter.P || kd != Constants.Shooter.D || kf != Constants.Shooter.F) {
            kp = Constants.Shooter.P;
            kd = Constants.Shooter.D;
            kf = Constants.Shooter.F;
            pid = Controller.pid(kp, 0, kd).plus(Controller.proportionalFeedforward(kf));
        }
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

        double deltaX = Math.sqrt((comps.follower.pose().x() - hivePos.x()) * (comps.follower.pose().x() - hivePos.x())
                                + (comps.follower.pose().y() - hivePos.y()) * (comps.follower.pose().y() - hivePos.y()));

        double alpha = Math.toRadians(Constants.Shooter.SHOOTER_ANGLE);
        /*
        * LUCKY YOU IM GONNA EXPLAIN MY MATH:
        * -0.5 * G * T^2 + V * SIN(A) * T + DELTA_Y = 0
        * T = (DELTA_X) / (V * COS(A))
        * (-G * DELTA_X^2) / (2 * V^2 * COS^2(A)) + (V * SIN(A) * DELTA_X) / (V * COS(A)) + DELTA_Y = 0
        * -G * DELTA_X^2 = (TAN(A) * DELTA_X - DELTA_Y) * 2 * V^2 * COS^2(A)
        * SOOOOOO
        * V = SQRT((-G * DELTA_X^2) / (2 * COS^2(A)(TAN(A) * DELTA_X - DELTA_Y)))
         */
        double targetV = Math.sqrt( (-Constants.Field.GRAV_ACC * deltaX * deltaX) / (2 * Math.cos(Math.toRadians(Constants.Shooter.SHOOTER_ANGLE)) * Math.cos(Math.toRadians(Constants.Shooter.SHOOTER_ANGLE))
                * (Math.tan(Math.toRadians(Constants.Shooter.SHOOTER_ANGLE)) * deltaX - Constants.Field.DELTA_HEIGHT))) * Constants.Shooter.AIR_RES_COMP_MULT; // Source: Just trust me bro


        double targetAngle = Math.atan2(hivePos.y() - comps.follower.pose().y(), hivePos.x() - comps.follower.pose().x());
        double targetVX = Math.cos(alpha) * targetV;
        double targetVY = Math.sin(alpha) * targetV;
        double targetVXX = Math.cos(targetAngle) * targetVX;
        double targetVXY = Math.sin(targetAngle) * targetVX;

        double vXX = targetVXX - comps.follower.velocity().vx;
        double vXY = targetVXY - comps.follower.velocity().vy;

        double v = Math.sqrt(vXX * vXX + vXY * vXY + targetVY * targetVY) + overrideOffset;

        shooter.setVelocity(pid.calculate(toRpm(v), toRpm(v) - shooter.getVelocity()));

    }

    private double toRpm(double v){
        return (v / ( 2 * Math.PI * Constants.Shooter.FLYWHEEL_RADIUS)) * 60;
    }

}
