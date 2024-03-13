package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;

import static edu.wpi.first.units.Units.*;

public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;

        // SmartDashboard keys
        public static final String kDriveSensitivity = "Drive Sensitivity";
        public static final String kTurnSensitivity = "Turn Sensitivity";
        public static final String kAutoTurn = "Auto Turn Sensitivity";

        // Joystick dead-band
        public static final double kJoystickDeadzone = 0.05;
    }

    public static class DIOConstants {
        /**
         * The DIO port for the climber limit switch
         */
        public static final int kClimberLimitSwitch = 1;
    }

    public static class AnalogInputConstants {
        public static final int kIntakeSensor = 0;
    }

    public static class CANConstants {
        // Differential Drive
        public static final int kLeftMainId = 1;
        public static final int kLeftSecondId = 2;
        public static final int kRightMainId = 3;
        public static final int kRightSecondId = 4;

        // Swerve Drive
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

        // Climber
        public static final int kClimberId = 10;

        // Ground Intake
        public static final int kGroundIntakeId = 11;

        // Indexer
        public static final int kIndexerMotor = 15;

        // Shooter
        public static final int kShooterMainId = 21;
        public static final int kShooterSecondaryId = 22;

        // Amp Guide
        public static final int kAmpGuideId = 30;
    }

    public static class GroundIntakeConstants {
        /**
         * What power to spin the intake motor with
         */
        public static final Measure<Dimensionless> kHighPower = Percent.of(0.50);

        /**
         * What power to spin the intake motor with
         */
        public static final Measure<Dimensionless> kLowPower = Percent.of(0.25);
    }

    public static class ClimberConstants {
        /**
         * What power to extend the climber motor with (percentage)
         */
        public static final Measure<Dimensionless> kExtendPower = Percent.of(0.75);

        /**
         * What power to retract the climber motor with (percentage)
         */
        public static final Measure<Dimensionless> kRetractPower = Percent.of(-1.0);

        /**
         * Climber extension soft limit (rotations)
         */
        public static final float kForwardLimit = 140f;

        public static final Measure<Dimensionless> kAutoHome = Percent.of(-10);
    }

    public static class ShooterConstants {
        /**
         * What velocity to shoot NOTES into the AMP with
         */
        public static final Measure<Velocity<Angle>> kShooterAmpSpeed = RotationsPerSecond.of(26);

        /**
         * Tolerance for the shooter to be considered at setpoint
         */
        public static final Measure<Velocity<Angle>> kShooterTolerance = RotationsPerSecond.of(2.0);

        /**
         * Amount of time it takes for the shooter to reach any setpoint
         * <p>
         * For the SPEAKER this measure gives us a consistent time-to-launch
         */
        public static final Measure<Time> kTimeout = Seconds.of(0.5);
    }

    public static class IndexerConstants {
        /**
         * The power to drive the intake motor towards the shooter (percentage)
         */
        public static final Measure<Dimensionless> kIndexerPower = Percent.of(0.5);
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
         * Efficiency: 0.75
         * <p>
         * speed = diameter * pi * gear ratio * efficiency * rpm / 60
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

    public static class AmpGuideConstants {
        /**
         * AMP Guide soft limit / AMP Guide scoring position (rotations)
         */
        public static final float kForwardLimit = 22.83f;

        /**
         * Tolerance to be considered "within setpoint". (rotations)
         * <p>
         * Note: we're off loading the PIDController to our Motor Controller, so this
         * tolerance has no effect on whether or not we're still commanding the AMP
         * Guide. This is value is only used to help build Command compositions.
         */
        public static final float kTolerance = 1.0f;
    }
}
