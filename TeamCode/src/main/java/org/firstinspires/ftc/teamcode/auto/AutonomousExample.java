package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.ArrayList;
import java.util.List;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Storage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
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
