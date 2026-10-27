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
    public DcMotorEx intake;
    public intakeState state = intakeState.TAKE_STATIC;
    public enum intakeState {
        TAKE_IN,
        TAKE_OUT,
        TAKE_STATIC
    }

    public Intake(HardwareMap hwm) {
        this.hardwareMap = hwm;

        this.intake = hardwareMap.get(DcMotorEx.class, "intake");
        this.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        this.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update() {
        switch (state){
            case TAKE_IN:
                intake.setPower(Constants.Intake.INTAKE_POWER);
                break;

            case TAKE_OUT:
                intake.setPower(Constants.Intake.OUTTAKE_POWER);
                break;

            case TAKE_STATIC:
                intake.setPower(Constants.Intake.STATIC_POWER);
                break;
        }
    }
}
