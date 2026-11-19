package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

@Configurable
public class Constants {
    @Configurable
    public static class Intake {
        public static double INTAKE_POWER = 0.8;
        public static double STATIC_POWER = 0;
        public static double OUTTAKE_POWER = -0.8;
    }

    @Configurable
    public static class Throughput {
        public static double INTHROUGH_POWER = 0.8;
        public static double STATIC_POWER = 0;
        public static double OUTTHROUGH_POWER = -0.8;
    }

    @Configurable
    public static class Limelight {
        public static double HEIGHT = 8.976378; // In inches the height from the floor to the lens
        public static double FORWARD_OFFSET = 7.677165; // In inches the offset of the limelight, forward is positive, backward is negative
        public static double LATERAL_OFFSET = 1.574803; // In inches the offset of the limelight, right is positive, left is negative
        public static double ANGLE = 65; // In degrees the angle between the vertical and the direction of the lens
    }

    @Configurable
    public static class Gate {
        public static double CLOSE_POS = 0.6;
        public static double OPEN_POS = 0.35;
    }

    @Configurable
    public static class Shooter {
        public static final double TICKS_PER_REV = 28;
        public static double SHOOTER_ANGLE = 75;
        public static double AIR_RES_COMP_MULT = 4.7;
        public static double FLYWHEEL_RADIUS = 0.1;
        public static double P = 20;
        public static double D = 0.5;
        public static double F = 0.0002;
        public static double GEAR = (double) 36 / 30;
        public static double OVERRIDE_CHANGE = 5;
    }

    @Configurable
    public static class Turret {
        public static double ENC_DIR_LEFT = -1;
        public static double ENC_DIR_RIGHT = 1; // might be wrong but idgaf
        public static double MIN_ANGLE = -315;          // soft limits, degrees, robot-relative (0 = facing front)
        public static double MAX_ANGLE = 315;
        public static double ZERO_OFFSET = 360; // this shi is measured

        public static final double RATIO_RIGHT = 5.0 / 6.0; // if turret moves 5 times encoder moves 6
        public static final double RATIO_LEFT = 4.0 / 5.0;
        public static double P = 0.01;
        public static double D = 0;
        public static double F = 0.00015;
        public static double OVERRIDE_CHANGE = 10;
    }

    @Configurable
    public static class Field {
        public static double GRAV_ACC = -9.81;
        public static double DELTA_HEIGHT = 1.39;
        public static Pose HIVE_POS_RED_DOWN = new Pose(57, 54);
        public static Pose HIVE_POS_RED_UP = new Pose(57, 93);
        public static Pose HIVE_POS_BLUE_DOWN = new Pose(84, 54);
        public static Pose HIVE_POS_BLUE_UP = new Pose(84, 93);
    }
}
