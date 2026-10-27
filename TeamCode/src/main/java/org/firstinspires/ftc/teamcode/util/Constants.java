package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

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
}
