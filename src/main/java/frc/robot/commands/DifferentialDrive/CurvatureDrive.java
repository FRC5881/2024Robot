package frc.robot.commands.DifferentialDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DifferentialDriveConstants;
import frc.robot.subsystems.DifferentialDriveSubsystem;

/**
 * Curvature drive command for differential drive robots.
 */
public class CurvatureDrive extends Command {
    private final DifferentialDriveSubsystem drive;
    private final DoubleSupplier throttleSupplier, curvatureSupplier;
    private final BooleanSupplier quickTurnSupplier;

    /**
     * Creates a new CurvatureDrive.
     *
     * @param drive     The drive subsystem this command will run on
     * @param throttle  The throttle value (forward/backward) as a percentage of max
     *                  speed
     * @param curve     The curvature value (turn) as a percentage of max rotation
     * @param quickTurn If true, the robot may turn in place
     */
    public CurvatureDrive(DifferentialDriveSubsystem drive, DoubleSupplier throttle, DoubleSupplier curve,
            BooleanSupplier quickTurn) {
        this.drive = drive;
        this.throttleSupplier = throttle;
        this.curvatureSupplier = curve;
        this.quickTurnSupplier = quickTurn;
        this.addRequirements(drive);
    }

    @Override
    public void execute() {
        double drive_sensitivity = SmartDashboard.getNumber("drive sensitivity", 1.0);
        double turn_sensitivity = SmartDashboard.getNumber("turn sensitivity", 1.0);

        double throttle = throttleSupplier.getAsDouble();

        double turn;
        if (quickTurnSupplier.getAsBoolean()) {
            turn = curvatureSupplier.getAsDouble();
        } else {
            turn = curvatureSupplier.getAsDouble() * throttle;
        }

        double vx = drive_sensitivity * throttle * DifferentialDriveConstants.MAX_SPEED;
        double omega = turn_sensitivity * turn * DifferentialDriveConstants.MAX_OMEGA;
        ChassisSpeeds speed = new ChassisSpeeds(vx, 0, omega);

        drive.drive(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
