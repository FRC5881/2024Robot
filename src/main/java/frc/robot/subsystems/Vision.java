package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    // APRIL Tags
    //
    // Absolute - PoseEstimator
    // Relative - GetVisionTarget(int id)
    //
    // NOTES
    //
    // Absolute - Pose2d getNearestNote()
    // Absolute - ArrayList<Pose2d> getAllNotes()
    // Absolute - void removeNotes(ArrayList<Pose2d> notes)
    // Absolute - Omniscient
    //
    // Relative - Optional<Rotation2d> yawNearestNote() - NEEDED
    // Relative - Optional<double> distanceNearestNote() - Later
    // Relative - boolean seeNote()
    //
    // DriveTrain : faceNote().andThen(driveForward())
    private final PhotonCamera camera;
    private static Vision instance = null;

    private Vision() {
        camera = new PhotonCamera("Logi_Webcam_C920e (1)");
    }

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    // Command cSkedaddle()
    //
    // boolean hasNoteTarget()

    public double getTargetDist() {
        PhotonPipelineResult result = camera.getLatestResult();
        boolean hasTargets = result.hasTargets();

        if (hasTargets == true) {
            PhotonTrackedTarget target = result.getBestTarget();

            double height = 3;
            // height of the camera above the ground
            double cameraSetAngle = 7;
            // cameraSetAngle is the angle between the vertical and 0 pitch of the camera

            double theta = target.getPitch() + cameraSetAngle;
            // theta should be zero when it points straight down
            // Angle between vertical and the note

            double tardist = (height / (Math.cos(theta))) * (Math.sin(theta));

            return tardist;
        }

        // List<PhotonTrackedTarget> targets = result.getTargets();
        // Currently unneccessary

        else {
            return -1;
        }
    }

    // Precondition: this command only makes sense if there is a target in frame
    public double getTargetYaw() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            double yaw = target.getYaw();
            return -yaw;
        }

        return 0;
    }
}
