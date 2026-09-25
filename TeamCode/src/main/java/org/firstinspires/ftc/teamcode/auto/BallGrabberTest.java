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
@Autonomous(name = "BallGrabberTest")
public class BallGrabberTest extends OpMode {

    public PoseFactory poseFac = PoseFactory.radians();
    private Follower follower;
    List<LynxModule> allHubs;
    public ElapsedTime Timer = new ElapsedTime();
    private Timer pathTimer, actionTimer, opmodeTimer;
    public Pose startPose = poseFac.of(8, 8, Math.toRadians(0));
    public ComponentShell comps;
    private TelemetryManager telemetryM;
    public ComponentShell.Alliance alliance;
    private Pose lastBallPose;


    public void autonomousPathUpdate() {
        List<Pose> ballPoses = comps.limelight.getBallPoses();
        if(!follower.isBusy() || pathTimer.seconds() > 5) {
            if(!ballPoses.isEmpty()) {
                Pose target = ballPoses.get(0);
                double heading = Math.atan2(follower.pose().y() - target.y(), follower.pose().x() - target.x());
                Path path = Paths.line(follower.pose(), target).linear(follower.pose().heading(), heading);
                follower.follow(path);
                pathTimer.reset();
                lastBallPose = target;
            }
            else {
                double heading = Math.atan2(follower.pose().y() - lastBallPose.y(), follower.pose().x() - lastBallPose.x());
                Path path = Paths.line(follower.pose(), lastBallPose).linear(follower.pose().heading(), heading);
                follower.follow(path);
                pathTimer.reset();
            }
        }
    }

    private double distance(Pose a, Pose b) {
        return Math.sqrt((a.x() - b.x()) * (a.x() - b.x()) + (a.y() - b.y()) * (a.y() - b.y()));
    }

    private boolean isDetecting(List<Pose> poses, Pose target) {
        for (Pose p : poses) {
            if (distance(p, target) < 5) {return true;}
        }
        return false;
    }

    private Pose closestPose(List<Pose> poses, Pose from) {
        Pose best = poses.get(0);
        double bestDist = distance(from, best);
        for(Pose p : poses) {
            if(distance(p, from) > bestDist) {
                best = p;
                bestDist = distance(p, from);
            }
        }
        return best;
    }

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.reset();
        pathTimer.reset();
        lastBallPose = startPose;

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        comps = new ComponentShell(hardwareMap, follower, telemetryM, alliance, poseFac, true);
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        follower.update();
        autonomousPathUpdate();

        comps.update();

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
