package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax secondaryShooterMotor;
    private final CANSparkMax mainShooterMotor;

    /**
     * adds can id to motors
     */
    public ShooterSubsystem() {
        mainShooterMotor = new CANSparkMax(Constants.CANConstants.kShooterId, MotorType.kBrushless);
        mainShooterMotor.restoreFactoryDefaults();
        mainShooterMotor.setSmartCurrentLimit(40);
        mainShooterMotor.setIdleMode(IdleMode.kBrake);

        secondaryShooterMotor = new CANSparkMax(Constants.CANConstants.kShooterIntakeId, MotorType.kBrushless);
        secondaryShooterMotor.restoreFactoryDefaults();
        secondaryShooterMotor.setSmartCurrentLimit(40);
        secondaryShooterMotor.setIdleMode(IdleMode.kBrake);

        mainShooterMotor.set(0);
        secondaryShooterMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooterRPM", mainShooterMotor.getEncoder().getVelocity());
    }

    /**
     * Sets speed of the shooter (percent)
     */
    public void setSpeed(double percentSpeed) {
        mainShooterMotor.set(percentSpeed);
        secondaryShooterMotor.set(percentSpeed);
    }

    public boolean isAtSetPoint() {
        // TODO:
        return true;
    }

    /**
     * Stops the shooter and intake
     */
    public void stop() {
        secondaryShooterMotor.stopMotor();
        mainShooterMotor.stopMotor();
    }

    /**
     * cSetSpeed is a helper command that sets the speed of the shooter
     */
    private Command cSetSpeed(double percentage) {
        return this.runOnce(() -> {
            this.setSpeed(percentage);
        });
    }

    /**
     * Spins the shooter up to low speed and then moves the NOTE up into the shooter
     */
    public Command cReadyLow() {
        return this.cSetSpeed(ShooterConstants.kShooterLowSpeed)
                .andThen(this.cWaitForSetPoint());
    }

    /**
     * Spins the shooter up to high speed and then moves the NOTE up into the
     * shooter
     */
    public Command cReadyHigh() {
        return this.cSetSpeed(ShooterConstants.kShooterHighSpeed)
                .andThen(this.cWaitForSetPoint());
    }

    public Command cStop() {
        return this.runOnce(this::stop);
    }

    /**
     * Command that ends when the shooter is at it's setpoint
     * 
     * @return the command
     */
    public Command cWaitForSetPoint() {
        // return Commands.waitUntil(this::isAtSetPoint);
        return Commands.waitSeconds(1);
    }

    /**
     * Spins the shooter and the intake in reverse to bring a NOTE into position
     */
    public Command cSourceIntake() {
        return this.cSetSpeed(-ShooterConstants.kShooterIntakePower)
                .andThen(Commands.idle())
                .finallyDo(this::stop);
    }
}
