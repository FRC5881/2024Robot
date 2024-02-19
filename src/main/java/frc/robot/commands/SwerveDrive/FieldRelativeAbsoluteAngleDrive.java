package frc.robot.commands.SwerveDrive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
            Supplier<Rotation2d> angle) {
        this.drive = drive;
        this.vxSupplier = vx;
        this.vySupplier = vy;
        this.angleSupplier = angle;
        this.MAX_SPEED = drive.getMaximumVelocity();
        this.MAX_OMEGA = drive.getMaximumAngularVelocity();
        this.angleController = new PIDController(0.2, 0, 0);
        this.angleController.enableContinuousInput(0, 1);
        this.addRequirements(drive);
    }

    public FieldRelativeAbsoluteAngleDrive(SwerveSubsystem drive, DoubleSupplier vx, DoubleSupplier vy,
            Rotation2d angle) {
        this(drive, vx, vy, () -> {
            return angle;
        });

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double drive_sensitivity = SmartDashboard.getNumber(OperatorConstants.kDriveSensitivity, 1.0);
        double turn_sensitivity = SmartDashboard.getNumber(OperatorConstants.kTurnSensitivity, 1.0);

        // Handle translation
        var vx = MAX_SPEED.times(vxSupplier.getAsDouble() * drive_sensitivity);
        var vy = MAX_SPEED.times(vySupplier.getAsDouble() * drive_sensitivity);

        // Handle Rotation
        Rotation2d currentAngle = drive.getHeading();
        double pidOutput = angleController.calculate(currentAngle.getRotations(), angleSupplier.get().getRotations());
        var omega = MAX_OMEGA.times(pidOutput * turn_sensitivity);

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
