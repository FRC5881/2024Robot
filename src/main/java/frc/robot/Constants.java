package frc.robot;

public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class CANConstants {
        public static final int kClimberId = 10;
        public static final int kShooterId = 21;
        public static final int kIntakeId = 22;
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

    public static class Shooter {
        /**
         * What power to load the shooter with (percentage)
         */
        public static final double kUpPower = 1.0;

        /**
         * What power to intake the note with (percentage)
         */
        public static final double kDownPower = 1.0;

        /**
         * What power to shoot high with (percentage)
         */
        public static final double kShootHighSpeed = 1.0;

        /**
         * What power to shoot low with (percentage)
         */
        public static final double kShootLowSpeed = 0.5;

        /**
         * What how long to wait till it is at speed to shoot high with (seconds)
         */
        public static final double shootHighTime = 3;

        /**
         * What how long to wait till it is at speed to shoot high with (seconds)
         */
        public static final double shootLowTime = 1.5;

        /**
         * What how long to wait till it is at speed to shoot high with (seconds)
         */
        public static final double shootIntakeTime = 3;

    }
}
