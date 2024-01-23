package frc.robot.commands.DifferentialDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DifferentialDriveConstants;
import frc.robot.subsystems.DifferentialDriveSubsystem;

/**
 * Arcade drive command for differential drive robots.
 */
public class ArcadeDrive extends Command {
    private final DifferentialDriveSubsystem drive;
    private final DoubleSupplier vxSupplier, omegaSupplier;

    /**
     * Creates a new ArcadeDrive.
     *
     * @param drive The drive subsystem this command will run on
     * @param vx    The vx value (forward/backward) as a percentage of max speed
     * @param omega The omega value (turn) as a percentage of max rotation rate
     */
    public ArcadeDrive(DifferentialDriveSubsystem drive, DoubleSupplier vx, DoubleSupplier omega) {
        this.drive = drive;
        this.vxSupplier = vx;
        this.omegaSupplier = omega;
        this.addRequirements(drive);
    }

    @Override
    public void execute() {
        double vx = vxSupplier.getAsDouble() * DifferentialDriveConstants.MAX_SPEED;
        double omega = omegaSupplier.getAsDouble() * DifferentialDriveConstants.MAX_OMEGA;

        ChassisSpeeds speed = new ChassisSpeeds(vx, 0, omega);
        drive.drive(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}