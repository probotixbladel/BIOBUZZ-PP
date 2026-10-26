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
public class Intake {
    private HardwareMap hardwareMap;
    private DcMotorEx intake;

    public Intake(HardwareMap hwm) {
        this.hardwareMap = hwm;

        this.intake = hardwareMap.get(DcMotorEx.class, "intake");
        this.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        this.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void takeIn() {
        intake.setPower(Constants.Intake.INTAKEPOWER);
    }

    public void takeStatic() {
        intake.setPower(Constants.Intake.STATICPOWER);
    }

    public void takeOut() {
        intake.setPower(Constants.Intake.OUTTAKEPOWER);
    }
}
