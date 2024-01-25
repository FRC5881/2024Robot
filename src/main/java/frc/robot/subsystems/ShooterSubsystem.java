package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    private final CANSparkMax shooterMotor;

    /**
     * adds can id to motors
     */
    public ShooterSubsystem() {
        intakeMotor = new CANSparkMax(Constants.CANConstants.kShooterIntakeId, MotorType.kBrushed);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(40);
        intakeMotor.setInverted(true);

        shooterMotor = new CANSparkMax(Constants.CANConstants.kShooterId, MotorType.kBrushless);
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setSmartCurrentLimit(40);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooterRPM", shooterMotor.getEncoder().getVelocity());
    }

    /**
     * Sets speed of the shooter (percent)
     */
    public void setSpeed(double percentSpeed) {
        shooterMotor.set(percentSpeed);
        intakeMotor.set(percentSpeed);
    }

    /**
     * Commands the intake send the NOTE up into the shooter
     */
    public void up() {
        intakeMotor.set(ShooterConstants.kIntakeUpPower);
    }

    /**
     * Commands the intake to take in a NOTE from the shooter
     */
    public void down() {
        intakeMotor.set(-ShooterConstants.kIntakeDownPower);
    }

    /**
     * Stops the shooter and intake
     */
    public void stop() {
        intakeMotor.stopMotor();
        shooterMotor.stopMotor();
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
     * Spins the shooter up to high speed and then moves the NOTE up into the
     * shooter
     */
    public Command cShootHigh() {
        return this.cSetSpeed(ShooterConstants.kShooterHighSpeed)
                .andThen(Commands.waitSeconds(0.75))
                .andThen(this.runOnce(this::up))
                .andThen(Commands.idle())
                .finallyDo(this::stop);
    }

    /**
     * Spins the shooter up to low speed and then moves the NOTE up into the shooter
     */
    public Command cShootLow() {
        return this.cSetSpeed(ShooterConstants.kShooterLowSpeed)
                .andThen(Commands.waitSeconds(0.5))
                .andThen(this.runOnce(this::up))
                .andThen(Commands.idle())
                .finallyDo(this::stop);
    }

    /**
     * Spins the shooter and the intake in reverse to bring a NOTE into position
     */
    public Command cSourceIntake() {
        return this.cSetSpeed(-ShooterConstants.kShooterIntakePower)
                .andThen(this.runOnce(this::down))
                .andThen(Commands.idle())
                .finallyDo(this::stop);
    }

    /**
     * Runs both the shooter and the intake at full speed
     */
    public Command cRunBoth() {
        return this.run(() -> {
            shooterMotor.set(1);
            intakeMotor.set(1);
        }).finallyDo(this::stop);
    }
}
