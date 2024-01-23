package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class CANConstants {
        public static final int kLeftMainId = 1;
        public static final int kLeftSecondId = 2;
        public static final int kRightMainId = 3;
        public static final int kRightSecondId = 4;

        public static final int kFrontLeftDriveId = 1;
        public static final int kFrontLeftAngleId = 5;
        public static final int kFrontRightDriveId = 2;
        public static final int kFrontRightAngleId = 6;
        public static final int kBackLeftDriveId = 3;
        public static final int kBackLeftAngleId = 7;
        public static final int kBackRightDriveId = 4;
        public static final int kBackRightAngleId = 8;

        public static final int kFrontLeftAzimuthEncoderId = 1;
        public static final int kFrontRightAzimuthEncoderId = 2;
        public static final int kBackLeftAzimuthEncoderId = 3;
        public static final int kBackRightAzimuthEncoderId = 4;

        public static final int kClimberId = 10;
        public static final int kGroundIntakeId = 11;

        public static final int kShooterId = 21;
        public static final int kShooterIntakeId = 22;
    }

    public static class IntakeConstants {
        /**
         * What power to spin the intake motor with (percentage)
         */
        public static final double kIntakePower = 1.0;
    }

    public static class ClimberConstants {
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

    public static class ShooterConstants {
        /**
         * The power to drive the intake motor towards the shooter (percentage)
         */
        public static final double kIntakeUpPower = 1.0;

        /**
         * The power to drive the intake motor away from the shooter (percentage)
         */
        public static final double kIntakeDownPower = 1.0;

        /**
         * What power to shoot high with (percentage)
         */
        public static final double kShooterHighSpeed = 1.0;

        /**
         * What power to shoot low with (percentage)
         */
        public static final double kShooterLowSpeed = 0.5;

        /**
         * What power to intake a NOTE with (percentage)
         */
        public static final double kShooterIntakePower = 0.25;
    }

    public static class DifferentialDriveConstants {
        /**
         * Maximum forward velocity of the drive train (mps).
         * <p>
         * Free speed: 5310 rpm
         * <p>
         * Effective gear ratio: 8.45:1
         * <p>
         * Wheel diameter: 6 inches
         * <p>
         * Efficency: 0.75
         * <p>
         * speed = diameter * pi * gear ratio * efficency * rpm / 60
         */
        public static final double MAX_SPEED = Units.inchesToMeters(6 * Math.PI * (1 / 8.45) * 0.75 * 5310 / 60);

        /**
         * Track width of the robot (meters).
         * <p>
         * TODO: Measure effective track width with sys-id
         */
        public static final double TRACK_WIDTH = Units.inchesToMeters(25);

        /**
         * Maximum angular velocity of the drive train (rad/s)
         */
        public static final double MAX_OMEGA = 2 * MAX_SPEED / TRACK_WIDTH;
    }
}
