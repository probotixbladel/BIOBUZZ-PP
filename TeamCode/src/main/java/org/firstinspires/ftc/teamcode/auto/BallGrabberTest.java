package org.firstinspires.ftc.teamcode.auto;



import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
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
import org.firstinspires.ftc.teamcode.util.PathFinder;

@Configurable
@Autonomous(name = "BallGrabberTest")
public class BallGrabberTest extends OpMode {
    public PathFinder pathFinder;
    public PoseFactory poseFac = PoseFactory.radians();
    private Follower follower;
    List<LynxModule> allHubs;
    public ElapsedTime Timer = new ElapsedTime();
    private Timer pathTimer, actionTimer, opmodeTimer, notDetectingTime;
    public Pose startPose = poseFac.of(12, 12, Math.toRadians(45));
    public ComponentShell comps;
    private TelemetryManager telemetryM;
    public ComponentShell.Alliance alliance = ComponentShell.Alliance.RED;
    private Pose currentTarget;
    private boolean wasDetecting = false;
    private Pose isDetecting = null;
    private boolean escapingBorder = true;
    private final Pose upGoal = poseFac.of(24, 120, Math.toRadians(-45));
    private final Pose downGoal = poseFac.of(24, 24, Math.toRadians(45));


    public void autonomousPathUpdate() {
        List<Pose> poses = comps.limelight.getBallPoses();
        if (!poses.isEmpty()) {
            Pose closestPose = closestPose(poses, follower.pose());
            if (currentTarget == null){
                currentTarget = closestPose;
            }
            isDetecting = isDetecting(poses, currentTarget);
            if (isDetecting == null) {
                if (notDetectingTime.seconds() > 0.5) {
                    currentTarget = closestPose;
                    pathTimer.reset();
                }
            }
            else {
                currentTarget = isDetecting;
            }
            if (wasDetecting && isDetecting == null) {
                notDetectingTime.reset();
            }
        }
        else if (!escapingBorder && notDetectingTime.seconds() < 0.5){
            if (follower.pose().y() > 72) {
                Path path = pathFinder.pathGenerator(follower.pose(), upGoal);
                follower.follow(path);
                escapingBorder = true;
                currentTarget = null;
            }
            else {
                Path path = pathFinder.pathGenerator(follower.pose(), downGoal);
                follower.follow(path);
                escapingBorder = true;
                currentTarget = null;

            }
        }

        if(!follower.isBusy() && currentTarget != null && pathTimer.seconds() > 1) {
            if(currentTarget.x() < 72) {
                //Path path = Paths.line(follower.pose(), currentTarget)
                //        .linear(follower.pose().heading(), Math.atan2(currentTarget.y() - follower.pose().y(), currentTarget.x() - follower.pose().x()));
                Path path = pathFinder.pathGenerator(follower.pose(), currentTarget);
                follower.follow(path);
                escapingBorder = false;
            }
            else {
                if(comps.follower.pose().x() > 50 && (comps.follower.pose().heading() < 0.25 * Math.PI || comps.follower.pose().heading() > 1.75 * Math.PI)) {
                    follower.hold(follower.pose().withHeading(Math.PI - follower.pose().heading()));
                }
                else {
                    if (follower.pose().y() > 72) {
                        Path path = pathFinder.pathGenerator(follower.pose(), upGoal);
                        follower.follow(path);
                        escapingBorder = true;
                        currentTarget = null;
                    }
                    else {
                        Path path = pathFinder.pathGenerator(follower.pose(), downGoal);
                        follower.follow(path);
                        escapingBorder = true;
                        currentTarget = null;

                    }

                }
            }
        }

        if (!escapingBorder) {
            if ((follower.pose().y() > 130 && follower.pose().heading() < 0.75 * Math.PI && follower.pose().heading() > 0.25 * Math.PI)
                    || (follower.pose().x() < 14 && follower.pose().y() > 72 && follower.pose().heading() < 1.25 * Math.PI && follower.pose().heading() > 0.75 * Math.PI)) {
                Path path = Paths.line(follower.pose(), upGoal).linear(follower.pose().heading(), upGoal.heading());
                follower.follow(path);
                currentTarget = null;
                escapingBorder = true;
            } else if ((follower.pose().y() < 14 && follower.pose().heading() < 1.75 * Math.PI && follower.pose().heading() > 1.25 * Math.PI)
                    || (follower.pose().x() < 14 && follower.pose().y() < 72 && follower.pose().heading() < 1.25 * Math.PI && follower.pose().heading() > 0.75 * Math.PI)) {
                Path path = Paths.line(follower.pose(), downGoal).linear(follower.pose().heading(), downGoal.heading());
                follower.follow(path);
                currentTarget = null;
                escapingBorder = true;
            }
        }

        wasDetecting = isDetecting != null;
    }

    private double distance(Pose a, Pose b) {
        return Math.sqrt((a.x() - b.x()) * (a.x() - b.x()) + (a.y() - b.y()) * (a.y() - b.y()));
    }

    private Pose isDetecting(List<Pose> poses, Pose target) {
        for (Pose p : poses) {
            if (distance(p, target) < 5) {return p;}
        }
        return null;
    }

    private Pose closestPose(List<Pose> poses, Pose from) {
        Pose best = poses.get(0);
        double bestDist = distance(from, best);
        for(Pose p : poses) {
            if(distance(p, from) < bestDist) {
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
        notDetectingTime = new Timer();
        opmodeTimer.reset();
        pathTimer.reset();

        follower = Constants.createFollower(hardwareMap);
        pathFinder = new PathFinder();
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

        telemetryM.debug("target: ", currentTarget != null ? currentTarget : "No target");
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
        notDetectingTime.reset();
    }

    @Override
    public void stop() {
        Storage.write(alliance, follower.pose());
    }
}
