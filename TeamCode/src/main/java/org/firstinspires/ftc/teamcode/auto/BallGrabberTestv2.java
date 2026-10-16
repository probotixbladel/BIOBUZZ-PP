package org.firstinspires.ftc.teamcode.auto;



import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
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
import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;

@Configurable
@Autonomous(name = "BallGrabberTestV2")
public class BallGrabberTestv2 extends OpMode {
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
    private Pose isDetecting = null;
    private boolean escapingBorder = true;
    private final Pose upGoal = poseFac.of(24, 120, Math.toRadians(-45));
    private final Pose downGoal = poseFac.of(24, 24, Math.toRadians(45));
    private PIDController turnPID;
    private double kp;
    private double kd;
    public static double p = 0;
    public static double d = 0;
    private boolean prevUp, prevDown, prevLeft, prevRight;


    @Override
    public void start() {
        opmodeTimer.reset();
        pathTimer.reset();
        notDetectingTime.reset();
        turnPID = Controller.pid(0, 0, 0);
    }


    public void autonomousPathUpdate() {
        /*if(kp != p || kd != d) {
            kp = p;
            kd = d;
            turnPID = Controller.pid(kp, 0, kd);
        }

        double[] closestBall = comps.limelight.getClosestBall();

        if(closestBall != null) {
            double tx = closestBall[0];
            double ty = closestBall[1];

            double turn = turnPID.calculate(0, -tx); // flip sign to match your original direction
            turn = Math.max(-0.6, Math.min(0.6, turn));
            follower.manual(new DrivePowers(0, 0, turn));
        }
        else {
            follower.manual(0, 0, 0);
        }
        telemetryM.debug("p", p);
        telemetryM.debug("d", d);*/

        double[] closestBall = comps.limelight.getClosestBall();
        if(closestBall != null && !follower.isBusy()) {
            double tx = closestBall[0];
            double ty = closestBall[1];
            telemetryM.debug("tx: ", tx);
            follower.hold(follower.pose().withHeading(follower.pose().heading() - Math.toRadians(tx)));
        }
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

        PanelsConfigurables.INSTANCE.refreshClass(this);
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
    public void stop() {
        Storage.write(alliance, follower.pose());
    }
}
