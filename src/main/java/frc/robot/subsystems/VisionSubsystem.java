package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    public VisionSubsystem() {
    }

    /**
     * Returns the most recent estimated robot pose from PhotonVision.
     * 
     * @return The most recent estimated robot pose from PhotonVision.
     */
    public Optional<EstimatedRobotPose> getRobotPose() {
        return Optional.empty();
    }
}
