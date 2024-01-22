package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot.RobotFrame;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
    private final Optional<VisionSubsystem> m_visionSubsystem;
    private final SwerveDrive m_swerveDrive;

    // Tracks the last time we received a vision measurement.
    private double m_lastVisionMeasurementTime = 0;

    /**
     * Creates a new SwerveSubsystem.
     * 
     * @param visionSubsystem The vision subsystem to use for pose estimation.
     * @throws IOException If the swerve module configuration file cannot be read.
     */
    public SwerveSubsystem(Optional<VisionSubsystem> visionSubsystem, RobotFrame bot) throws IOException {
        m_visionSubsystem = visionSubsystem;

        String swerveDir;
        switch (bot) {
            case COMP:
                swerveDir = "compbot";
                break;
            case M1C2:
                swerveDir = "m1c2";
                break;
            default:
                throw new IOException("SwerveSubsystem is only configured for COMP and M1C2");
        }

        SwerveParser parser = new SwerveParser(new File(Filesystem.getDeployDirectory(), swerveDir));

        // https://www.swervedrivespecialties.com/products/mk4-swerve-module
        // L1 free speed is allegedly 12.5 ft/s
        double maxSpeed = Units.feetToMeters(12.5);
        // Steering gear ratio of the MK4 is 12.8:1
        double angleMotorConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8, 1);
        // Drive gear ratio for the L1 is 8.14:1
        double driveMotorConversion = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 8.14, 1);

        m_swerveDrive = parser.createSwerveDrive(maxSpeed, angleMotorConversionFactor, driveMotorConversion);
    }

    @Override
    public void periodic() {
        if (m_visionSubsystem.isPresent()) {
            updateVision();
        }
    }

    /**
     * Updates the swerve drive with the latest vision measurement.
     * 
     * <p>
     * 
     * **NOTE:** This method assumes that the vision subsystem is present.
     */
    private void updateVision() {
        Optional<EstimatedRobotPose> position = m_visionSubsystem.get().getRobotPose();
        if (position.isPresent()) {
            Pose2d pose = position.get().estimatedPose.toPose2d();
            double timestamp = position.get().timestampSeconds;

            // If this is a new measurement, then add it to the pose history.
            if (timestamp > m_lastVisionMeasurementTime) {
                m_lastVisionMeasurementTime = timestamp;
                m_swerveDrive.addVisionMeasurement(pose, timestamp);
            }
        }
    }

    /**
     * Field-relative swerve drive.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        m_swerveDrive.driveFieldOriented(chassisSpeeds);
    }

    /**
     * Gets the current pose of the robot.
     * 
     * @return The estimated position (rotation + translation) of the robot.
     */
    public Pose2d getPose() {
        return m_swerveDrive.getPose();
    }

    /**
     * Returns the current gyro angle.
     * 
     * @return The current gyro angle as a Rotation2d.
     */
    public Rotation2d getHeading() {
        return m_swerveDrive.getYaw();
    }

    /**
     * Resets the robot's position on the field.
     * 
     * @param pose The new position of the robot.
     */
    public void resetPose(Pose2d pose) {
        m_swerveDrive.resetOdometry(pose);
    }
}
