package frc.robot.subsystems.StraightShooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class StraightShooterSubsystem extends SubsystemBase {
    StraightShooterIO io;

    public StraightShooterSubsystem() {
        if (Robot.isReal()) {
            io = new StraightShooterIOReal();
        } else {
            io = new StraightShooterIOSim();
        }
    }

    @Override
    public void periodic() {
        if (Robot.isSimulation()) {
            ((StraightShooterIOSim) io).update(0.02);
        }

        SmartDashboard.putNumber("/StraightShooter/Top Left Shooter RPM", io.getTopLeftVelocity());
        SmartDashboard.putNumber("/StraightShooter/Top Right Shooter RPM", io.getTopRightVelocity());
        SmartDashboard.putNumber("/StraightShooter/Bottom Left Shooter RPM", io.getBottomLeftVelocity());
        SmartDashboard.putNumber("/StraightShooter/Bottom Right Shooter RPM", io.getBottomRightVelocity());
    }

    // Drive all 4 motors at 12 volts
    public Command cRunAt12V() {
        SmartDashboard.putNumber("/StraightShooter/Shooter Voltage", 0);
        return runEnd(() -> {
            SmartDashboard.putNumber("/StraightShooter/Shooter Voltage", 12);
            io.setVoltage(12, 12, 12, 12);
        }, () -> {
            SmartDashboard.putNumber("/StraightShooter/Shooter Voltage", 0);
            io.setVoltage(0, 0, 0, 0);
        });
    }
}
