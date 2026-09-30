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

@Configurable
public class Intake {
    private HardwareMap hardwareMap;
    private DcMotorEx intake;
    public static double intakePower = 1;
    public static double staticPower = 0;

    public Intake(HardwareMap hwm) {
        this.hardwareMap = hwm;

        this.intake = hardwareMap.get(DcMotorEx.class, "intake");
        this.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        this.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void takeIn() {
        intake.setPower(intakePower);
    }

    public void takeStatic() {
        intake.setPower(staticPower);
    }
}
