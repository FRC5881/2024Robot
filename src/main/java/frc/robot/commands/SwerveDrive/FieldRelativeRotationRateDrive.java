package frc.robot.commands.SwerveDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Command to drive a swerve drive robot with field relative translation and a
 * rotation rate.
 */
public class FieldRelativeRotationRateDrive extends Command {
    private final SwerveSubsystem drive;
    private final DoubleSupplier vxSupplier, vySupplier, omegaSupplier;

    /**
     * Creates a new FieldRelativeRotationRateDrive.
     * 
     * @param drive The drive subsystem this command will run on
     * @param vx    The vx value (forward/backward) as a percentage of max speed
     * @param vy    The vy value (left/right) as a percentage of max speed
     * @param omega The omega value (rotate) as a percentage of max rotation rate
     */
    public FieldRelativeRotationRateDrive(SwerveSubsystem drive, DoubleSupplier vx, DoubleSupplier vy,
            DoubleSupplier omega) {
        this.drive = drive;
        this.vxSupplier = vx;
        this.vySupplier = vy;
        this.omegaSupplier = omega;
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double drive_sensitivity = SmartDashboard.getNumber("drive sensitivity", 1.0);
        double turn_sensitivity = SmartDashboard.getNumber("turn sensitivity", 1.0);

        double vx = drive_sensitivity * vxSupplier.getAsDouble() * SwerveDriveConstants.MAX_SPEED;
        double vy = drive_sensitivity * vySupplier.getAsDouble() * SwerveDriveConstants.MAX_SPEED;
        // TODO: Depend on RobotFrame
        double omega = turn_sensitivity * omegaSupplier.getAsDouble() * SwerveDriveConstants.MAX_OMEGA_M1C1;

        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
        drive.drive(speeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
