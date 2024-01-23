package frc.robot.commands.SwerveDrive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Command to drive a swerve drive robot with field relative translation and an
 * absolute angle.
 * 
 * This uses a PID controller to control the angle of the robot.
 */
public class FieldRelativeAbsoluteAngleDrive extends Command {
    private final SwerveSubsystem drive;
    private final DoubleSupplier vxSupplier, vySupplier;
    private final Supplier<Rotation2d> angleSupplier;

    /**
     * Creates a new FieldRelativeAbsoluteAngleDrive.
     * 
     * @param drive          The drive subsystem this command will run on
     * @param vx             The vx value (forward/backward) as a percentage of max
     *                       speed
     * @param vy             The vy value (left/right) as a percentage of max speed
     * @param angle          The angle of the robot
     * @param angleTolerance The tolerance of the angle
     */
    public FieldRelativeAbsoluteAngleDrive(SwerveSubsystem drive, DoubleSupplier vx, DoubleSupplier vy,
            Supplier<Rotation2d> angle, Rotation2d angleTolerance) {
        this.drive = drive;
        this.vxSupplier = vx;
        this.vySupplier = vy;
        this.angleSupplier = angle;
        this.addRequirements(drive);
    }

    // TODO: Implement this
}
