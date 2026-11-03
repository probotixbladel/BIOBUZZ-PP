package org.firstinspires.ftc.teamcode.components;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Detector {
    private DigitalChannel fish;
    public boolean detecting;
    public Detector(HardwareMap hwm) {
        fish = hwm.get(DigitalChannel.class, "fish");
        fish.setMode(DigitalChannel.Mode.INPUT);
    }

    public void update() {
        detecting = !fish.getState();
    }
}
