package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
@Configurable
@Disabled
@Autonomous(name = "AutonomousExample")
public class AutonomousExample extends AutonomousSample {
    @Override
    public void init() {
        // to start you need to define the startpose and alliance and the run the init from the autonomous sample
        startPose = poseFac.of(0, 0, Math.toRadians(0));
        alliance = ComponentShell.Alliance.BLUE;
        super.init();

        //here you see different functions to call

        MoveCurve(poseFac.of(0, 0, Math.toRadians(0)), poseFac.of(0, 0, Math.toRadians(0)), 10, true);
        Pause(10);
        MoveLinear(poseFac.of(0, 0, Math.toRadians(0)), 2, true);
        End();
    }
}
