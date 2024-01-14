package frc.robot;

public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class CANConstants {
        public static final int kClimberId = 10;
        public static final int kIntakeId = 20;
        public static final int kShooterFlywheelId = 21;
        public static final int kShooterIntakeId = 22;

    }

    public static class Intake {
        /**
         * What power to spin the intake motor with (percentage)
         */
        public static final double kIntakePower = 1.0;
    }

    public static class Climber {
        /**
         * What power to extend the climber motor with (percentage)
         */
        public static final double kExtendPower = 1.0;

        /**
         * What power to retract the climber motor with (percentage)
         */
        public static final double kRetractPower = 1.0;

        /**
         * Climber extension soft limit (rotations)
         */
        public static final float kForwardLimit = 100.0f;
    }
}
