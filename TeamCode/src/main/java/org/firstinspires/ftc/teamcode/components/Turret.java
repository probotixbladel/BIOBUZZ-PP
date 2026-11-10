package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.util.Constants;

public class Turret {
    public CRServo turret1;
    public CRServo turret2;
    private ComponentShell comps;
    public Turret(HardwareMap hwm, ComponentShell comps) {
        this.turret1 = hwm.get(CRServo.class, "turret1");
        this.turret2 = hwm.get(CRServo.class, "turret2");
        turret2.setDirection(DcMotorSimple.Direction.REVERSE);
        this.comps = comps;
    }

    public void update() {
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


    }
}
