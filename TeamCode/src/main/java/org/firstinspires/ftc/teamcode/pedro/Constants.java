package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("leftFront");
                c.backLeftName.set("leftBack");
                c.frontRightName.set("rightFront");
                c.backRightName.set("rightBack");

                c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("pinpoint");
                c.xPodOffset.set(-1.2003576473926936);
                c.yPodOffset.set(-5.810135969026821);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.35425091045652374);
                Controller secondaryTranslationalForward = Controller.proportional(0.13088624565502477);
                Controller primaryTranslationalLateral = Controller.proportional(0.7784864343901082);
                Controller secondaryTranslationalLateral = Controller.proportional(0.2876299359665106);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.021883622640181302));
                c.brake.set(Controller.proportionalFeedforward(0.018601079244154106));

                c.headingFeedback.set(Controller.proportional(6.919500391961786));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05002833626088184, 0.020330170058661265));

                c.linearBrakeCoefficients.set(Matrix.diag(0.08723166412454723, 0.023685283681408868));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0023471919150935194, 0.004589357268343463));

                c.maxAchievableForwardVelocity.set(53.94524784573766);
                c.maxAchievableStrafeVelocity.set(42.26927279837583);
                c.naturalForwardDeceleration.set(35.58907127654011);
                c.naturalStrafeDeceleration.set(64.44127628088692);
            }
    );

    public static Follower createFollower(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}