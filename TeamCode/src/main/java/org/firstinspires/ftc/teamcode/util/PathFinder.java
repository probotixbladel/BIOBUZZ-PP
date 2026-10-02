package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class PathFinder {
    private PoseFactory poseFac = PoseFactory.radians();
    private double redTrussX = 48;
    private double redTrussYUp = 96;
    private double redTrussYDown = 48;
    private final Pose flower1 = poseFac.of(0, 48, 0);
    private final Pose flower2 = poseFac.of(48, 144, 0);
    private final Pose trussControlUp = poseFac.of(48, 144, 0);
    private final Pose trussControlDown = poseFac.of(48, 0, 0);
    public PathFinder(){

    }

    public Path pathGenerator(Pose from, Pose to) {
        if ((from.x() < redTrussX && to.x() < redTrussX)
                || (from.x() > redTrussX && to.x() > redTrussX)
                || (from.y() > redTrussYUp && to.y() > redTrussYUp)
                || (from.y() < redTrussYDown && to.y() < redTrussYDown)) {

            if (from.x() < flower1.x() + 24 && to.x() < flower1.x() + 24
                && ((from.y() < flower1.y() && to.y() > flower1.y())
                || (from.y() > flower1.y() && to.y() < flower1.y()))) {
                return Paths.curve(from, poseFac.of(flower1.x() + 24, flower1.y(), 0), to).linear(from, to);
            }
            if (from.y() > flower2.y() - 24 && to.y() > flower2.y() - 24
                    && ((from.x() < flower2.x() && to.x() > flower2.x())
                    || (from.x() > flower2.x() && to.x() < flower2.x()))) {
                return Paths.curve(from, poseFac.of(flower2.x() + 24, flower2.y(), 0), to).linear(from, to);
            }

            return Paths.line(from, to).linear(from, to);
        }
        double avX = (from.x() + to.x()) / 2;
        if (avX > 72) {
            return Paths.curve(from, trussControlUp, to).linear(from, to);
        }
        return Paths.curve(from, trussControlDown, to).linear(from, to);
    }
}
