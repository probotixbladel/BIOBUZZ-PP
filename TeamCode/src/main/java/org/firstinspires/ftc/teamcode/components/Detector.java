package org.firstinspires.ftc.teamcode.components;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Detector {
    private DigitalChannel fish1;
    private DigitalChannel fish1;
    private DigitalChannel fish2;
    private DigitalChannel fish3;
    private DigitalChannel fish4;
    private DigitalChannel fish5;
    public int ballNum = 0;
    public boolean detecting1;
    public boolean detecting2;
    public boolean detecting3;
    public boolean detecting4;
    public boolean detecting5;
    public Detector(HardwareMap hwm) {
        fish1 = hwm.get(DigitalChannel.class, "fish");
        fish1.setMode(DigitalChannel.Mode.INPUT);

        fish2 = hwm.get(DigitalChannel.class, "fish");
        fish2.setMode(DigitalChannel.Mode.INPUT);

        fish3 = hwm.get(DigitalChannel.class, "fish");
        fish3.setMode(DigitalChannel.Mode.INPUT);

        fish4 = hwm.get(DigitalChannel.class, "fish");
        fish4.setMode(DigitalChannel.Mode.INPUT);

        fish5 = hwm.get(DigitalChannel.class, "fish");
        fish5.setMode(DigitalChannel.Mode.INPUT);
    }

    public void update() {
        detecting1 = !fish1.getState();
        detecting2 = !fish1.getState();
        detecting3 = !fish1.getState();
        detecting4 = !fish1.getState();
        detecting5 = !fish1.getState();

        ballNum = findBallNum();
    }

    private int findBallNum(){
        int ballNumTemp = 0;
        if(detecting1) ballNumTemp++;
        if(detecting2) ballNumTemp++;
        if(detecting3) ballNumTemp++;
        if(detecting4) ballNumTemp++;
        if(detecting5) ballNumTemp++;

        return ballNumTemp;
    }
}
