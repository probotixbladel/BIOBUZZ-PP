package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Constants {
    public static class Intake {
        public static double INTAKEPOWER = 0.8;
        public static double STATICPOWER = 0;
        public static double OUTTAKEPOWER = -0.8;
    }

    public static class Throughput {
        public static double INTHROUGHPOWER = 0.8;
        public static double STATICPOWER = 0;
        public static double OUTTHROUGHPOWER = -0.8;
    }

    public static class Limelight {
        public static double HEIGHT = 8.976378; // In inches the height from the floor to the lens
        public static double FORWARDOFFSET = 7.677165; // In inches the offset of the limelight, forward is positive, backward is negative
        public static double LATERALOFFSET = 1.574803; // In inches the offset of the limelight, right is positive, left is negative
        public static double ANGLE = 65; // In degrees the angle between the vertical and the direction of the lens
    }
}
