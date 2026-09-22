package org.firstinspires.ftc.teamcode.drive;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
import org.firstinspires.ftc.teamcode.components.Storage;
@Configurable
@TeleOp(name="Red Reset", group="Linear OpMode")
public class RedReset extends LinearOpMode {
    private static PoseFactory poseFac = PoseFactory.radians();
    public static Pose startingPose = poseFac.of(8, 8, Math.toRadians(90)); //See ExampleAuto to understand how to use this //x = 9 without triangles

    @Override
    public void runOpMode() {
        Storage.write(ComponentShell.Alliance.RED, startingPose);
    }
}
