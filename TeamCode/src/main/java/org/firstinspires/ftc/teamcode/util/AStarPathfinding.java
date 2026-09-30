package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.api.PoseFactory;

public class AStarPathfinding {
    public PoseFactory poseFac = PoseFactory.radians();
    private Node node;
    private Node nodeDuo;

    public void Start() {
        node = new Node(0, 0, poseFac.of(0, 0,0), poseFac.of(0, 0, 0));
        nodeDuo = new Node(0, 0, poseFac.of(0, 0, 0), poseFac.of(0, 0, 0));
    }
}
