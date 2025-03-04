package frc.robot;

import frc.robot.Constants.Setpoints;

public final class Vars {
    public static class Positions {
        public static final class RobotStates {
            public static Setpoints.kLiftPosition kLiftState;
            public static Setpoints.kHarpoonPosition kHarpoonState;
        }
    }

    public static class Throttles {
        public static double kAlgaeIntakeThrottle = 1;
        public static double kCoralIntakeThrottle = 0.5;
        public static double kHarpoonThrottle = 1;
    }
}
