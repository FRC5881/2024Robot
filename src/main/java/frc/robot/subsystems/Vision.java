package frc.robot.subsystems;

import java.time.Clock;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // This need to update with each tick... how?
    private double m_last_timestamp = 0;
    private Rotation2d m_last_heading = new Rotation2d();

    // private double m_last_timestamp = System.currentTimeMillis();

    // Precondition: this command only makes sense if there is a target in frame
    public Optional<Rotation2d> getTargetYaw(Rotation2d robotHeading) {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.getTimestampSeconds() == m_last_timestamp) {
            return Optional.of(m_last_heading);
        }

        m_last_timestamp = result.getTimestampSeconds();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            double yaw = target.getYaw();

            Rotation2d yawR = new Rotation2d(Math.toRadians(-yaw));

            return Optional.of(yawR);
        }

        else {
            Rotation2d temp = new Rotation2d(0);
            return Optional.of(temp);
        }
    }

    public boolean hasTarget() {
        PhotonPipelineResult result = camera.getLatestResult();
        return result.hasTargets();
    }

    public Rotation2d getSmoothYaw() {
        // LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
        // SmartDashboard.putNumber("/Vision/Raw Yaw", getTargetYaw().getDegrees());
        // double smooth = filter.calculate(getTargetYaw().getDegrees());
        // SmartDashboard.putNumber("/Vision/Smooth Yaw", smooth);
        // return Rotation2d.fromDegrees(smooth);

        return getTargetYaw();
    }

    public double getTargetPitch() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            double pitch = target.getPitch();

            return pitch;
        }

        else {
            double temp = 90;
            return temp;
        }
    }
}
