package frc.robot.commands.SwerveDrive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveDriveConstants;
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

    private final PIDController angleController;

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

        this.angleController = new PIDController(0.2, 0, 0);
        this.angleController.enableContinuousInput(0, 1);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Handle Rotation
        Rotation2d currentAngle = drive.getHeading();
        double anglePower = angleController.calculate(currentAngle.getRotations(), angleSupplier.get().getRotations());
        // TODO: RobotFrame
        double omega = anglePower * SwerveDriveConstants.MAX_OMEGA_M1C1;

        // Handle translation
        double drive_sensitivity = SmartDashboard.getNumber("drive sensitivity", 1.0);
        double vx = drive_sensitivity * vxSupplier.getAsDouble() * SwerveDriveConstants.MAX_SPEED;
        double vy = drive_sensitivity * vySupplier.getAsDouble() * SwerveDriveConstants.MAX_SPEED;

        // Drive!
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
