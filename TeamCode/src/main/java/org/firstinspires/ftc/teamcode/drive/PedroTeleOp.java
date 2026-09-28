package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.pedro.Constants.createFollower;

import android.annotation.SuppressLint;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.teamcode.components.ComponentShell;
import org.firstinspires.ftc.teamcode.components.Storage;
import org.firstinspires.ftc.teamcode.util.DriveByWire;

import java.util.List;

@Configurable
@TeleOp(name="PedroTeleop")
public class PedroTeleOp extends OpMode{
    private PoseFactory poseFac = PoseFactory.radians();
    public static boolean singlePlayer = false;
    private Follower follower;
    private ComponentShell Comps;
    List<LynxModule> allHubs;
    private TelemetryManager telemetryM;
    public static ComponentShell.Alliance alliance;
    private DriveByWire driveByWire;
    private boolean robotCentric = false;
    public static Pose startingPose;

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Storage.Data data = Storage.read();

        follower = createFollower(hardwareMap);
        startingPose = data.storedPose;
        follower.setPose(startingPose == null ? poseFac.of(8, 8, Math.toRadians(90)) : startingPose);
        follower.update();
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        alliance = data.storedAlliance;
        Comps = new ComponentShell(hardwareMap, follower, telemetryM, alliance, poseFac, singlePlayer);
        driveByWire = new DriveByWire(Comps);

    }

    @Override
    public void start() {

    }
    @SuppressLint("SuspiciousIndentation")
    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        if (gamepad1.dpad_up) {
            robotCentric = !robotCentric;
        }

        double[] driveInputs = driveByWire.getScaledInputs(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        if (!robotCentric) {
            follower.manual(driveInputs[1], driveInputs[0], driveInputs[2]);
        }
        else{
            switch(alliance) {
                case BLUE:
                    follower.manual(ManualDrive.fieldCentric(
                            driveInputs[1], driveInputs[0], driveInputs[2],
                            follower.pose().heading(),
                            0
                    ));
                    break;
                case RED:
                    follower.manual(ManualDrive.fieldCentric(
                            driveInputs[1], driveInputs[0], driveInputs[2],
                            follower.pose().heading(),
                            Math.PI
                    ));
            }
        }

        Comps.updateTeleop(gamepad1, gamepad2);
        follower.update();
        telemetryM.update();
    }

    @Override
    public void stop() {}
}
