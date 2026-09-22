package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.ComponentShell;

@Configurable
public class DriveByWire {
    private ComponentShell Comps;
    public static double translationalCurveExponent = 2.0;
    public static double rotationalCurveExponent = 1.2;
    public static double translationalGear = 1;
    public static double rotationalGear = 1;

    public DriveByWire (ComponentShell comps) {this.Comps = comps;}

    public double[] getScaledInputs (double x, double y, double yaw) {
        double length = Math.sqrt(x * x + y * y);
        double scale = Math.pow(length, translationalCurveExponent) * translationalGear;
        yaw *= Math.pow(yaw, rotationalCurveExponent) * rotationalGear;
        x *= scale;
        y *= scale;

        return new double[] {x, y, yaw};
    }
}
