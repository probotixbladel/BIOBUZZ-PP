package org.firstinspires.ftc.teamcode.components;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Constants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


public class Limelight {
    private PoseFactory poseFac;
    private Limelight3A limelight;
    private TelemetryManager telemetryM;
    private Follower follower;
    public static double nectarRad = 1.8; // In inches the radius of the nectar
    public static double pollenRad = 1.4; // In inches the radius of the pollen

    public Limelight (HardwareMap hwm, TelemetryManager tm, Follower flw, PoseFactory pf) {
        this.telemetryM = tm;
        this.limelight = hwm.get(Limelight3A.class, "Limelight");
        this.follower = flw;
        this.poseFac = pf;

        limelight.pipelineSwitch(0);
        limelight.start();
        limelight.pause();
    }

    public void update() {

    }

    public double[] getClosestBall() {
        limelight.start();
        LLResult result = limelight.getLatestResult();
        List<Pose> poses = new ArrayList<>();

        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

            String className = detections.get(0).getClassName();      // e.g. "yellow_pollen", "red_nectar", "blue_nectar"
            double tx = detections.get(0).getTargetXDegrees();          // left-right offset
            double ty = detections.get(0).getTargetYDegrees();          // up-down offset
            double area = detections.get(0).getTargetArea();            // rough size/distance proxy

            limelight.pause();
            return new double[]{tx, ty};
        }
        telemetryM.addData("Limelight", "No detections");
        limelight.pause();
        return null;
    }

    public List<Pose> getBallPoses() {
        limelight.start();
        LLResult result = limelight.getLatestResult();
        List<Pose> poses = new ArrayList<>();

        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

            for (LLResultTypes.DetectorResult d : detections) {
                String className = d.getClassName();      // e.g. "yellow_pollen", "red_nectar", "blue_nectar"
                double tx = d.getTargetXDegrees();          // left-right offset
                double ty = d.getTargetYDegrees();          // up-down offset
                double area = d.getTargetArea();            // rough size/distance proxy

                double yDisplacement;
                if (className.contains("_pollen")) {
                    yDisplacement = (Constants.Limelight.HEIGHT - pollenRad) * Math.tan(Math.toRadians(Constants.Limelight.ANGLE) + Math.toRadians(ty)) + pollenRad + Constants.Limelight.FORWARD_OFFSET;
                }
                else {
                    yDisplacement = (Constants.Limelight.HEIGHT - nectarRad) * Math.tan(Math.toRadians(Constants.Limelight.ANGLE) + Math.toRadians(ty)) + nectarRad + Constants.Limelight.FORWARD_OFFSET;
                }
                double xDisplacement = yDisplacement * Math.tan(Math.toRadians(tx)) + Constants.Limelight.LATERAL_OFFSET;

                double x = follower.pose().x() - xDisplacement * Math.sin(follower.pose().heading()) + yDisplacement * Math.cos(follower.pose().heading());
                double y = follower.pose().y() + xDisplacement * Math.cos(follower.pose().heading()) + yDisplacement * Math.sin(follower.pose().heading());

                poses.add(poseFac.of(x, y, 0));

                //telemetryM.addData(className, String.format("x=%.1f y=%.1f area=%.2f", x, y, area));
            }
        } else {
            telemetryM.addData("Limelight", "No detections");
        }
        limelight.pause();
        return poses;
    }
}
