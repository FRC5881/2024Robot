package frc.robot.commands.DifferentialDrive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DifferentialDriveConstants;
import frc.robot.subsystems.DifferentialDriveSubsystem;

/**
 * Tank drive command for differential drive robots.
 * <p>
 * Tank drive controls the left and right sides of the robot independently.
 */
public class TankDrive extends Command {
    private final DifferentialDriveSubsystem drive;
    private final DoubleSupplier leftSupplier, rightSupplier;

    /**
     * Creates a new TankDrive.
     *
     * @param drive The drive subsystem this command will run on
     * @param left  The left value (forward/backward) as a percentage of max speed
     * @param right The right value (forward/backward) as a percentage of max speed
     */
    public TankDrive(DifferentialDriveSubsystem drive, DoubleSupplier left, DoubleSupplier right) {
        this.drive = drive;
        this.leftSupplier = left;
        this.rightSupplier = right;
        this.addRequirements(drive);
    }

    @Override
    public void execute() {
        double left = leftSupplier.getAsDouble() * DifferentialDriveConstants.MAX_SPEED;
        double right = rightSupplier.getAsDouble() * DifferentialDriveConstants.MAX_SPEED;
        drive.drive(left, right);
    }
}
