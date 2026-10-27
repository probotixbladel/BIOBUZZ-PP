package org.firstinspires.ftc.teamcode.drive;
import static org.firstinspires.ftc.teamcode.pedro.Constants.createFollower;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
import org.firstinspires.ftc.teamcode.components.Storage;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.DriveByWire;

import java.util.List;

@Configurable
@TeleOp(name="Puppit")
public class Puppit extends OpMode {
    private PoseFactory poseFac = PoseFactory.radians();
    private Follower follower;
    private ComponentShell Comps;
    private TelemetryManager telemetryM;

    // VARIABLES TO TUNE:
    public static double INTAKE_POWER;
    public static double THROUGH_POWER;
    public static double GATE_POS;


    @SuppressLint("SuspiciousIndentation")
    @Override
    public void init() {
        follower = createFollower(hardwareMap);
        follower.update();
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Comps = new ComponentShell(hardwareMap, follower, telemetryM, ComponentShell.Alliance.BLUE, poseFac, false);

        INTAKE_POWER = Constants.Intake.STATIC_POWER;
        THROUGH_POWER = Constants.Throughput.STATIC_POWER;
        GATE_POS = Constants.Gate.CLOSE_POS;

    }


    @SuppressLint("SuspiciousIndentation")
    @Override
    public void loop() {
        Comps.intake.intake.setPower(INTAKE_POWER);
        Comps.through.through.setPower(THROUGH_POWER);
        Comps.gate.gate.setPosition(GATE_POS);
        telemetryM.addData("Intake: ", INTAKE_POWER);
        telemetryM.addData("Through: ", THROUGH_POWER);
        telemetryM.addData("Gate: ", GATE_POS);
    }
}
