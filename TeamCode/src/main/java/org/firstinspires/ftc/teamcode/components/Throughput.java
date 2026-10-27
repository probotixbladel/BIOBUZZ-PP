package org.firstinspires.ftc.teamcode.components;
import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.util.Constants;

@Configurable
public class Throughput {
    private HardwareMap hardwareMap;
    public DcMotorEx through;
    public throughState state = throughState.STATIC_THROUGH;
    public enum throughState{
        STATIC_THROUGH,
        IN_THROUGH,
        OUT_THROUGH
    }

    public Throughput(HardwareMap hwm) {
        this.hardwareMap = hwm;

        this.through = hardwareMap.get(DcMotorEx.class, "through");
        this.through.setDirection(DcMotorSimple.Direction.REVERSE);
        this.through.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        switch(state) {
            case STATIC_THROUGH:
                through.setPower(Constants.Throughput.STATIC_POWER);
                break;

            case IN_THROUGH:
                through.setPower(Constants.Throughput.INTHROUGH_POWER);
                break;

            case OUT_THROUGH:
                through.setPower(Constants.Throughput.OUTTHROUGH_POWER);
                break;
        }
    }
}
