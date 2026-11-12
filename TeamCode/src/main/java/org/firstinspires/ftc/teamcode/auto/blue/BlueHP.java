package org.firstinspires.ftc.teamcode.auto.blue;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutonomousSample;
import org.firstinspires.ftc.teamcode.components.ComponentShell;

@Configurable
@Autonomous(name = "BlueHP")
public class BlueHP extends AutonomousSample {
    @Override
    public void init() {
        // to start you need to define the startpose and alliance and the run the init from the autonomous sample
        startPose = poseFac.of(144 - 8, 144 - 132, Math.toRadians(-90));
        alliance = ComponentShell.Alliance.RED;
        super.init();

        Pause(10);
        Shoot(0, 5);
        MoveLinear(poseFac.of(144 - 8, 144 - 94, Math.toRadians(-90)), 2, true);
        MoveLinear(poseFac.of(144 - 24, 144 - 120, Math.toRadians(180)), 2, true);
        Shoot(0, 5);
        MoveLinear(poseFac.of(144 - 8, 144 - 120, Math.toRadians(180)), 2, true);
        End();
    }
}
