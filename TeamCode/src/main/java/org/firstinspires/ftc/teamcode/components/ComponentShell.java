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
    public TelemetryManager telemetryM;
    public Alliance alliance;
    public HardwareMap hardwareMap;
    public Intake intake;
    public Throughput through;
    public Gate gate;
    public Detector detector;
    public Shooter shooter;
    public Turret turret;
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
        this.telemetryM = tm;
        this.singlePlayer = single;
        this.limelight = new Limelight(hwm, tm, flw, pf);
        this.intake = new Intake(hwm);
        this.through = new Throughput(hwm);
        this.gate = new Gate(hwm);
        this.detector = new Detector(hwm);
        this.turret = new Turret(hwm, this);
        this.shooter = new Shooter(hwm, this);
    }

    @SuppressLint("SuspiciousIndentation")
    public void update() {

        limelight.update();
        telemetryM.addData("detecting: ", detector.findBallNum());
        telemetryM.addData("Pos= ", follower.pose());
        telemetryM.addData("al: ", alliance);
        telemetryM.addData("Turret angle: ", turret.angle());
        telemetryM.addData("Turret targetangle: ", turret.targetAngle());
        telemetryM.addData("Turret velCmd: ", turret.velCmd());
        telemetryM.addData("Turretvel A", turret.servoVelA());
        telemetryM.addData("Turretvel B", turret.servoVelB());

        intake.update();
        through.update();
        detector.update();
        turret.update();
        shooter.update();
    }

    public void updateTeleop(Gamepad gamepad1, Gamepad gamepad2) {
        this.update();

        if(gamepad2.left_trigger < 0.8) {
            if(detector.findBallNum() < 4) {
                intake.state = Intake.intakeState.TAKE_IN;
            }
        }
        else {
            intake.state = Intake.intakeState.TAKE_OUT;
        }

        if(gamepad2.a) {
            gate.openGate();
        }
        else {
            gate.closeGate();
        }
        if(detector.findBallNum() < 4){
            through.state = Throughput.throughState.IN_THROUGH;
        }
        else{
            through.state = Throughput.throughState.STATIC_THROUGH;
        }
    }
}
