package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.paths.Path;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.paths.curves.Curve;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Angle;

import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.math.Vector2D;

import com.pedropathing.paths.curves.Curve;

@Configurable
public class PathFinder {
    private PoseFactory poseFac = PoseFactory.radians();
    private double redTrussX = 48;
    private double redTrussYUp = 96;
    private double redTrussYDown = 48;
    private final Pose flower1 = poseFac.of(0, 48, 0);
    private final Pose flower2 = poseFac.of(48, 144, 0);
    private final Pose trussControlUp = poseFac.of(48, 130, 0);
    private final Pose trussControlDown = poseFac.of(48, 0, 0);
    public PathFinder(){

    }



    public static Interpolator lookAt(Vector2D target){
        return (Curve curve, double t) -> Angle.normalize(
                target.minus(curve.get(t)).theta()
        );
    }

    public Path pathGenerator(Pose from, Pose to) {
        if ((from.x() < redTrussX && to.x() < redTrussX)
                || (from.x() > redTrussX && to.x() > redTrussX)
                || (from.y() > redTrussYUp && to.y() > redTrussYUp)
                || (from.y() < redTrussYDown && to.y() < redTrussYDown)) {

            if (from.x() < flower1.x() + 24 && to.x() < flower1.x() + 24
                && ((from.y() < flower1.y() && to.y() > flower1.y())
                || (from.y() > flower1.y() && to.y() < flower1.y()))) {
                return Paths.curve(from, poseFac.of(flower1.x() + 24, flower1.y(), 0), to).heading(lookAt(to.toVector2D()));
            }
            if (from.y() > flower2.y() - 24 && to.y() > flower2.y() - 24
                    && ((from.x() < flower2.x() && to.x() > flower2.x())
                    || (from.x() > flower2.x() && to.x() < flower2.x()))) {
                return Paths.curve(from, poseFac.of(flower2.x() + 24, flower2.y(), 0), to).heading(lookAt(to.toVector2D()));
            }

            return Paths.line(from, to).heading(lookAt(to.toVector2D()));
        }
        double avY = (from.y() + to.y()) / 2;
        if (avY > 72) {
            return Paths.curve(from, trussControlUp, to).heading(lookAt(to.toVector2D()));
        }
        return Paths.curve(from, trussControlDown, to).heading(lookAt(to.toVector2D()));
    }
}
