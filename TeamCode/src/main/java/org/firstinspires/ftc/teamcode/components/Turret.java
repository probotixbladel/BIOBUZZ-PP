package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    public Servo turret;
    private ComponentShell comps;
    public Turret(HardwareMap hwm, ComponentShell comps) {
        this.turret = hwm.get(Servo.class, "turret");
        this.comps = comps;
    }
}
