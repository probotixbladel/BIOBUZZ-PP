package org.firstinspires.ftc.teamcode.auto.red;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutonomousSample;
import org.firstinspires.ftc.teamcode.components.ComponentShell;

@Configurable
@Autonomous(name = "RedHP")
public class RedHP extends AutonomousSample {
    @Override
    public void init() {
        // to start you need to define the startpose and alliance and the run the init from the autonomous sample
        startPose = poseFac.of(8, 132, Math.toRadians(-90));
        alliance = ComponentShell.Alliance.RED;
        super.init();

        Pause(10);
        Shoot(0, 5);
        MoveLinear(poseFac.of(8, 94, Math.toRadians(-90)), 2, true);
        MoveLinear(poseFac.of(24, 120, Math.toRadians(0)), 2, true);
        Shoot(0, 5);
        MoveLinear(poseFac.of(8, 120, Math.toRadians(0)), 2, true);
        End();
    }
}
