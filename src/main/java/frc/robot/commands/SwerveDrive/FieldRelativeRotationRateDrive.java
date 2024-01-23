package frc.robot.commands.SwerveDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
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

    // TODO: Implement this
}
