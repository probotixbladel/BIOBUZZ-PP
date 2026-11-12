package org.firstinspires.ftc.teamcode.auto.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.auto.AutonomousSample;
import org.firstinspires.ftc.teamcode.components.ComponentShell;
@Configurable
@Autonomous(name = "BlueGardenEgo")
public class BlueGardenEgo extends AutonomousSample {
    @Override
    public void init() {
        // to start you need to define the startpose and alliance and the run the init from the autonomous sample
        startPose = poseFac.of(144 - 31, 144 - 8, Math.toRadians(0));
        alliance = ComponentShell.Alliance.BLUE;
        super.init();


        Shoot(0, 5);
        MoveLinear(poseFac.of(144 - 8, 144 - 8, Math.toRadians(0)), 2, true);
        MoveCurve(poseFac.of(144 - 24, 144 - 120, Math.toRadians(180 - 200)), poseFac.of(144 - 48, 144 - 24, 0) ,4, true);
        Shoot(0, 5);
        MoveCurve(poseFac.of(144 - 8, 144 - 96, Math.toRadians(0)), poseFac.of(144 - 6, 144 - 127, 0), 2, true);
        MoveLinear(poseFac.of(144 - 25, 144 - 120, Math.toRadians(0)), 2, true);
        Shoot(0, 5);
        MoveLinear(poseFac.of(144 - 8, 144 - 120, Math.toRadians(0)), 2, true);
        End();
    }
}
