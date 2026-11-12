package org.firstinspires.ftc.teamcode.auto.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.auto.AutonomousSample;
import org.firstinspires.ftc.teamcode.components.ComponentShell;
@Configurable
@Autonomous(name = "RedGardenEgo")
public class RedGardenEgo extends AutonomousSample {
    @Override
    public void init() {
        // to start you need to define the startpose and alliance and the run the init from the autonomous sample
        startPose = poseFac.of(31, 8, Math.toRadians(180));
        alliance = ComponentShell.Alliance.RED;
        super.init();


        Shoot(0, 5);
        MoveLinear(poseFac.of(8, 8, Math.toRadians(180)), 2, true);
        MoveCurve(poseFac.of(24, 120, Math.toRadians(200)), poseFac.of(48, 24, 0) ,4, true);
        Shoot(0, 5);
        MoveCurve(poseFac.of(8, 96, Math.toRadians(180)), poseFac.of(6, 127, 0), 2, true);
        MoveLinear(poseFac.of(25, 120, Math.toRadians(180)), 2, true);
        Shoot(0, 5);
        MoveLinear(poseFac.of(8, 120, Math.toRadians(180)), 2, true);
        End();
    }
}
