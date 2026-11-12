package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

@Configurable
public class Constants {
    public static class Intake {
        public static double INTAKE_POWER = 0.8;
        public static double STATIC_POWER = 0;
        public static double OUTTAKE_POWER = -0.8;
    }

    public static class Throughput {
        public static double INTHROUGH_POWER = 0.8;
        public static double STATIC_POWER = 0;
        public static double OUTTHROUGH_POWER = -0.8;
    }

    public static class Limelight {
        public static double HEIGHT = 8.976378; // In inches the height from the floor to the lens
        public static double FORWARD_OFFSET = 7.677165; // In inches the offset of the limelight, forward is positive, backward is negative
        public static double LATERAL_OFFSET = 1.574803; // In inches the offset of the limelight, right is positive, left is negative
        public static double ANGLE = 65; // In degrees the angle between the vertical and the direction of the lens
    }

    public static class Gate {
        public static double CLOSE_POS = 0;
        public static double OPEN_POS = 0;
    }

    public static class Shooter {
        public static double SHOOTER_ANGLE = 75;
        public static double AIR_RES_COMP_MULT = 1.1;
        public static double FLYWHEEL_RADIUS = 0.1;
        public static double P = 1;
        public static double D = 0;
        public static double F = 1;
    }

    public static class Turret {
        // Paste these inside your existing Constants.Turret class (public static, same as P_A etc.).
// You can delete the old P_A, D_A, P_B, D_B: the new code no longer uses them.
// All numbers are STARTING POINTS, tune them in the order given in the message.

        // --- position loop: turret degrees of error -> turret deg/s command ---
        public static double POS_P = 6.0;
        public static double POS_D = 0.0;

        // --- speed loops (one per servo): servo deg/s of error -> power ---
        public static double VEL_P_A = 0.001;
        public static double VEL_D_A = 0.0;
        public static double VEL_P_B = 0.001;
        public static double VEL_D_B = 0.0;

        // --- feedforward per servo ---
// kV: power per deg/s. About 1 / (servo's max deg/s). Example: 0.12 s per 60 deg = 500 deg/s -> 0.002
// kS: power needed just to start moving (static friction)
        public static double KV_A = 0.002;
        public static double KV_B = 0.002;
        public static double KS_A = 0.03;
        public static double KS_B = 0.03;

        // --- limits and geometry ---
        public static double MAX_TURRET_VEL = 150;      // deg/s, keep low for the first tests
        public static double MIN_ANGLE = -200;          // soft limits, degrees, robot-relative (0 = facing front)
        public static double MAX_ANGLE = 200;
        public static double ZERO_OFFSET = 0;           // vernier angle when the turret faces robot front
        public static double OMEGA_FF = 1.0;            // 1 = counter-rotate against robot spin, 0 = off

        // --- signs: must be +1 or -1 so BOTH encoder angles INCREASE when the turret turns counter-clockwise ---
        public static int ENC_DIR_A = 1;
        public static int ENC_DIR_B = 1;

        public static final double RATIO_A = 6.0 / 5.0;   // encoder A turns per turret turn
        public static final double RATIO_B = 5.0 / 4.0;   // encoder B turns per turret turn
        public static final double VERNIER_GAIN = 1.0 / (RATIO_B - RATIO_A);   // = 20
        public static final double VEL_FILTER = 0.3;      // 0..1, higher = less smoothing of measured speed
    }

    public static class Field {
        public static double GRAV_ACC = 9.81;
        public static double DELTA_HEIGHT = 1.39;
        public static Pose HIVE_POS = new Pose(58, 54);
    }
}
