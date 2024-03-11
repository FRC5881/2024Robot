package frc.robot.commands.SwerveDrive;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Command to drive a swerve drive robot with field relative translation and an
 * absolute angle.
 * 
 * This uses a PID controller to control the angle of the robot.
 */
public class FieldRelativeAbsoluteAngleDrive extends Command {
    private final SwerveSubsystem drive;
    private final Measure<Velocity<Distance>> MAX_SPEED;
    private final Measure<Velocity<Angle>> MAX_OMEGA;
    private final Supplier<Translation2d> translationSupplier;
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
    public FieldRelativeAbsoluteAngleDrive(SwerveSubsystem drive, Supplier<Translation2d> translationSupplier,
            Supplier<Rotation2d> angle) {
        this.drive = drive;
        this.translationSupplier = translationSupplier;
        angleSupplier = angle;

        MAX_SPEED = drive.getMaximumVelocity();
        MAX_OMEGA = drive.getMaximumAngularVelocity();

        addRequirements(drive);
    }

    public FieldRelativeAbsoluteAngleDrive(SwerveSubsystem drive, Supplier<Translation2d> translationSupplier,
            Rotation2d angle) {
        this(drive, translationSupplier, () -> {
            return angle;
        });
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double drive_sensitivity = SmartDashboard.getNumber(OperatorConstants.kDriveSensitivity, 1.0);
        double kP = SmartDashboard.getNumber(OperatorConstants.kAutoTurn, 1.0);

        // Handle translation
        Translation2d translation = translationSupplier.get();
        var vx = MAX_SPEED.times(translation.getX() * drive_sensitivity);
        var vy = MAX_SPEED.times(translation.getY() * drive_sensitivity);

        // Handle Rotation
        Rotation2d currentAngle = drive.getHeading();
        double error = angleSupplier.get().getRotations() - currentAngle.getRotations();
        double errorBound = MathUtil.inputModulus(error, 0.5, -0.5);
        var omega = MAX_OMEGA.times(kP * errorBound);

        // Drive!
        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
        drive.driveFieldRelative(speeds);
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
