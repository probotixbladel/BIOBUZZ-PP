package org.firstinspires.ftc.teamcode.components;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.bylazar.telemetry.TelemetryManager;

@Configurable
public class ComponentShell {
    private PoseFactory poseFac;
    public boolean singlePlayer;
    public Follower follower;
    public TelemetryManager telemeryM;
    public Alliance alliance;
    public HardwareMap hardwareMap;
    public Intake intake;
    public Limelight limelight;
    public enum Alliance {
        BLUE,
        RED
    }

    public ComponentShell (HardwareMap hwm, Follower flw, TelemetryManager tm, Alliance al, PoseFactory pf, boolean single) {
        this.alliance = al;
        this.poseFac = pf;
        this.hardwareMap = hwm;
        this.follower = flw;
        this.telemeryM = tm;
        this.singlePlayer = single;
        this.limelight = new Limelight(hwm, tm, flw, pf);
        this.intake = new Intake(hwm);
        //this.intake = new Intake(hwm);
    }

    @SuppressLint("SuspiciousIndentation")
    public void update() {

        //intake.Update();

        limelight.update();
        telemeryM.addData("Pos= ", follower.pose());
    }

    public void updateTeleop(Gamepad gamepad1, Gamepad gamepad2) {

        this.update();

        if(gamepad1.right_trigger > 0.2) {
            intake.takeIn();
        }
        else {
            intake.takeStatic();
        }
    }
}
