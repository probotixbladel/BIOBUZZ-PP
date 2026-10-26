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
    private DcMotorEx through;
    public static double inthroughPower = 0.8;
    public static double staticthroughPower = 0;

    public Throughput(HardwareMap hwm) {
        this.hardwareMap = hwm;

        this.through = hardwareMap.get(DcMotorEx.class, "through");
        this.through.setDirection(DcMotorSimple.Direction.REVERSE);
        this.through.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void throughIn() {
        through.setPower(Constants.Throughput.INTHROUGHPOWER);
    }

    public void throughStatic() {
        through.setPower(Constants.Throughput.STATICPOWER);
    }

    public void throughOut() {
        through.setPower(Constants.Throughput.OUTTHROUGHPOWER);
    }
}
