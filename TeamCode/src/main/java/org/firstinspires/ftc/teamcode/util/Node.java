package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

public class Node {
    public double p;
    public double q;
    public double f;
    public Pose pose;
    public Pose originPose;
    public Node(double p, double q, Pose pose, Pose ogPose) {
        this.p = p;
        this.q = q;
        this.f = p + q;
        this.pose = pose;
        this.originPose = ogPose;
    }
}
