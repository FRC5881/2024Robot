package frc.robot.commands.SwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Drive command that merges two driver controllers into one command.
 * <p>
 * The "driver" has field-relative translation control while the copilot has
 * robot-relative control.
 * <p>
 * Both drivers prefer rotation rate control.
 */
public class TwoPlayerDriveCommand extends Command {
    SwerveSubsystem drive;
    CommandPS5Controller driverController;
    CommandPS5Controller copilotController;

    public TwoPlayerDriveCommand(SwerveSubsystem drive, CommandPS5Controller driverController, CommandPS5Controller copilotController) {
        this.drive = drive;
        this.driverController = driverController;
        this.copilotController = copilotController;
    }

    @Override
    public void initialize() {
        if (!SmartDashboard.containsKey("/Operator/Driver/Drive Sensitivity")) {
            SmartDashboard.putNumber("/Operator/Driver/Drive Sensitivity", 1.0);
            SmartDashboard.setPersistent("/Operator/Driver/Drive Sensitivity");
        }
        if (!SmartDashboard.containsKey("/Operator/Driver/Turn Sensitivity")) {
            SmartDashboard.putNumber("/Operator/Driver/Turn Sensitivity", 1.0);
            SmartDashboard.setPersistent("/Operator/Driver/Turn Sensitivity");
        }
        if (!SmartDashboard.containsKey("/Operator/Copilot/Drive Sensitivity")) {
            SmartDashboard.putNumber("/Operator/Copilot/Drive Sensitivity", 1.0);
            SmartDashboard.setPersistent("/Operator/Copilot/Drive Sensitivity");
        }
        if (!SmartDashboard.containsKey("/Operator/Copilot/Turn Sensitivity")) {
            SmartDashboard.putNumber("/Operator/Copilot/Turn Sensitivity", 1.0);
            SmartDashboard.setPersistent("/Operator/Copilot/Turn Sensitivity");
        }
    }

    @Override
    public void execute() {
        double drive_sensitivity = SmartDashboard.getNumber("/Operator/Driver/Drive Sensitivity", 1.0);
        double turn_sensitivity = SmartDashboard.getNumber("/Operator/Driver/Turn Sensitivity", 1.0);

        double copilot_drive_sensitivity = SmartDashboard.getNumber("/Operator/Copilot/Drive Sensitivity", 1.0);
        double copilot_turn_sensitivity = SmartDashboard.getNumber("/Operator/Copilot/Turn Sensitivity", 1.0);

        double driver_x = -deadzone(driverController.getLeftY());
        double driver_y = -deadzone(driverController.getLeftX());
        double driver_omega = -deadzone(driverController.getRightX());

        double copilot_x = -deadzone(copilotController.getLeftY());
        double copilot_y = -deadzone(copilotController.getLeftX());
        double copilot_omega = -deadzone(copilotController.getRightX());

        var driver = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
            drive_sensitivity * driver_x,
            drive_sensitivity * driver_y,
            turn_sensitivity * driver_omega
        ), drive.getHeading());

        var combined = new ChassisSpeeds(
            (copilot_drive_sensitivity * copilot_x) + driver.vxMetersPerSecond,
            (copilot_drive_sensitivity * copilot_y) + driver.vyMetersPerSecond,
            (copilot_turn_sensitivity * copilot_omega) + driver.omegaRadiansPerSecond
        );

        drive.driveRobotRelative(combined);
    }

    private static double deadzone(double x) {
        return Math.abs(x) < OperatorConstants.kJoystickDeadzone ? 0 : x;
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
