package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    private final VisionSubsystem m_visionSubsystem;

    public SwerveSubsystem(VisionSubsystem visionSubsystem) {
        m_visionSubsystem = visionSubsystem;
    }

    /**
     * Field-relative swerve drive.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        // TODO: drive the robot
    }

    /**
     * Gets the current pose of the robot.
     * 
     * @return The estimated position (rotation + translation) of the robot.
     */
    public Pose2d getPose() {
        // TODO: get actual pose
        return new Pose2d();
    }

    /**
     * Returns the current gyro angle.
     * 
     * @return The current gyro angle as a Rotation2d.
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }
}
