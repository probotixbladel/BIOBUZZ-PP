package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Gate {
    public Servo gate;
    public gateState state = gateState.CLOSED;
    public enum gateState {
        OPENED,
        CLOSED
    }
    public Gate(HardwareMap hwm) {
        this.gate = hwm.get(Servo.class, "gate");
    }

    public void openGate() {
        gate.setPosition(Constants.Gate.OPEN_POS);
        state = gateState.OPENED;
    }

    public void closeGate() {
        gate.setPosition(Constants.Gate.CLOSE_POS);
        state = gateState.CLOSED;
    }
}
