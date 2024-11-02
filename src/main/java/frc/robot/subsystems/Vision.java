package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Vision implements Subsystem {
    /**
     * Returns Object Detection data of NOTES in frame
     */
    private final PhotonCamera intakeCamera;
    /**
     * Used for Pose Estimation
     */
    private final PhotonCamera shooterCamera;

    public Vision() {
        intakeCamera = new PhotonCamera("intake");
        shooterCamera = new PhotonCamera("shooter");
    }

    public PhotonPipelineResult getLatestTags() {
        return shooterCamera.getLatestResult();
    }

    public Optional<Transform2d> getBestNote() {
        PhotonPipelineResult result = intakeCamera.getLatestResult();

        Optional<PhotonTrackedTarget> bestTarget = Optional.empty();;
        double bestDistance = Double.MAX_VALUE;
        for (PhotonTrackedTarget target : result.getTargets()) {
            // The best target is the one closest to the bottom center of the frame
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            double distance = yaw * yaw + pitch * pitch;

            if (distance < bestDistance) {
                bestTarget = Optional.of(target);
                bestDistance = distance;
            }
        }

        return bestTarget.map(this::getTransformToTarget);
    }

    public Transform2d getTransformToTarget(PhotonTrackedTarget target) {
        Rotation2d yaw = Rotation2d.fromDegrees(target.getYaw());
        double distance = PhotonUtils.calculateDistanceToTargetMeters(
            Units.inchesToMeters(24), 
            0, 
            Units.degreesToRadians(-15), 
            target.getPitch()
        );

        SmartDashboard.putNumber("/Vision/Distance", distance);
        SmartDashboard.putNumber("/Vision/Yaw", target.getYaw());

        return new Transform2d(
            new Translation2d(yaw.getCos() * distance, yaw.getSin() * distance),
            yaw
        );
    }
}
