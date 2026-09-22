package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;

import java.util.ArrayList;
import java.util.List;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.utils.Timer;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
import org.firstinspires.ftc.teamcode.components.Storage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Configurable
@Disabled
@Autonomous(name = "AutonomousSample")
public class AutonomousSample extends OpMode {
    /*
    This is AutonomousSample, the idea is that every autonomous inherits this class
    to call functions in the order the robot should perform them
    so we can make autonomouses easily.
    */

    public PoseFactory poseFac = PoseFactory.radians();
    private Follower follower;
    List<LynxModule> allHubs;
    public ElapsedTime Timer = new ElapsedTime();
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose examplePose = new Pose(17.43, 120.37, Math.toRadians(-36));
    public Pose startPose;
    public ComponentShell comps;
    public int Shots = 0;
    private TelemetryManager telemetryM;
    public ComponentShell.Alliance alliance;
    private boolean stop = false;
    private Pose lastPose;
    private int currentStepDrawn = 0;
    private int currentStepPlayed = 0;
    private List<List<Object>> steps = new ArrayList<>();
    public enum Actions {
        MOVE,
        FOLLOWPATH,
        PAUSE,
        END
    }

    public void MoveLinear(Pose to, float time, boolean waitTilFinished) {
        List<Object> step = steps.get(currentStepDrawn);
        Path path = Paths.line(lastPose, to).linear(lastPose, to);
        lastPose = to;
        step.set(0, Actions.MOVE);
        step.set(1, path);
        currentStepDrawn++;
        step = steps.get(currentStepDrawn);
        step.set(0, Actions.FOLLOWPATH);
        step.set(1, time);
        step.set(2, waitTilFinished);
        currentStepDrawn++;
    }

    public void MoveCurve(Pose to, Pose control, float time, boolean waitTilFinished) {
        List<Object> step = steps.get(currentStepDrawn);
        Path path = Paths.curve(lastPose, control, to).linear(lastPose, to);
        lastPose = to;
        step.set(0, Actions.MOVE);
        step.set(1, path);
        currentStepDrawn++;
        step = steps.get(currentStepDrawn);
        step.set(0, Actions.FOLLOWPATH);
        step.set(1, time);
        step.set(2, waitTilFinished);
        currentStepDrawn++;
    }

    public void Pause(float time) {
        List<Object> step = steps.get(currentStepDrawn);
        step.set(0, Actions.PAUSE);
        step.set(1, time);
        currentStepDrawn++;
    }

    public void End() {
        steps.get(currentStepDrawn).set(0, Actions.END);
        currentStepDrawn++;
    }


    public void autonomousPathUpdate() {
        List<Object> step = steps.get(currentStepPlayed);
        switch((Actions) step.get(0)) {
            case PAUSE:
                if (pathTimer.seconds() > (double) step.get(1)) {
                    pathTimer.reset();
                    currentStepPlayed++;
                }
                break;

            case MOVE:
                follower.follow( (Path) step.get(1));
                pathTimer.reset();
                currentStepPlayed++;
                break;

            case FOLLOWPATH:
                if((!follower.isBusy() || !(boolean) step.get(2)) && pathTimer.seconds() > (double) step.get(1)) { // step 1 is the max time and step 2 is the waitTilFinished bool
                    pathTimer.reset();
                    currentStepPlayed++;
                }
                break;

            case END:
                stop = true;
                break;

        }
    }

    @Override
    public void init() {
        stop = false;
        currentStepDrawn = 0;
        currentStepPlayed = 0;
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.reset();
        pathTimer.reset();

        follower = Constants.createFollower(hardwareMap);
        //follower.setStartingPose(startPose); TODO: fix this shii
        lastPose = startPose;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        comps = new ComponentShell(hardwareMap, follower, telemetryM, alliance, poseFac, true);
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        follower.update();
        if(!stop) {
            autonomousPathUpdate();
        }
        telemetryM.debug("Shots", Shots);

        comps.update();

        telemetryM.debug("path state", pathState);
        telemetryM.debug("x", follower.pose().x());
        telemetryM.debug("y", follower.pose().y());
        telemetryM.debug("heading", follower.pose().heading());
        telemetryM.debug("Timer: ", Timer.seconds());
        telemetryM.debug("isBusy: ", follower.isBusy());
        telemetryM.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        pathTimer.reset();
    }

    @Override
    public void stop() {
        Storage.write(alliance, follower.pose());
    }
}
