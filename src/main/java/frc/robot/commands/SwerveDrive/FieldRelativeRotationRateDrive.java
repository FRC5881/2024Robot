package frc.robot.commands.SwerveDrive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
 * Command to drive a swerve drive robot with field relative translation and a
 * rotation rate.
 */
public class FieldRelativeRotationRateDrive extends Command {
    private final SwerveSubsystem drive;
    private final Measure<Velocity<Distance>> MAX_SPEED;
    private final Measure<Velocity<Angle>> MAX_OMEGA;
    private final DoubleSupplier omegaSupplier;
    private final Supplier<Translation2d> translationSupplier;

    /**
     * Creates a new FieldRelativeRotationRateDrive.
     * 
     * @param drive The drive subsystem this command will run on
     * @param vx    The vx value (forward/backward) as a percentage of max speed
     * @param vy    The vy value (left/right) as a percentage of max speed
     * @param omega The omega value (rotate) as a percentage of max rotation rate
     */
    public FieldRelativeRotationRateDrive(SwerveSubsystem drive, Supplier<Translation2d> translationSupplier,
            DoubleSupplier omega) {
        this.drive = drive;
        this.translationSupplier = translationSupplier;
        this.omegaSupplier = omega;

        MAX_SPEED = drive.getMaximumVelocity();
        MAX_OMEGA = drive.getMaximumAngularVelocity();
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double drive_sensitivity = SmartDashboard.getNumber(OperatorConstants.kDriveSensitivity, 1.0);
        double turn_sensitivity = SmartDashboard.getNumber(OperatorConstants.kTurnSensitivity, 1.0);

        Translation2d translation = translationSupplier.get();
        var vx = MAX_SPEED.times(translation.getX() * drive_sensitivity);
        var vy = MAX_SPEED.times(translation.getY() * drive_sensitivity);
        var omega = MAX_OMEGA.times(-1 * omegaSupplier.getAsDouble() * turn_sensitivity);

        SmartDashboard.putNumber("vx", vx.baseUnitMagnitude());
        SmartDashboard.putNumber("vy", vy.baseUnitMagnitude());
        SmartDashboard.putNumber("omega", omega.baseUnitMagnitude());

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
