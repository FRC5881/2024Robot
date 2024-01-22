package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    private final CANSparkMax shooterMotor;

    /**
     * adds can id to motors
     */
    public ShooterSubsystem() {
        intakeMotor = new CANSparkMax(Constants.CANConstants.kShooterIntakeId, MotorType.kBrushed);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setSmartCurrentLimit(20);
        intakeMotor.setInverted(true);
        intakeMotor.burnFlash();

        shooterMotor = new CANSparkMax(Constants.CANConstants.kShooterId, MotorType.kBrushless);
        shooterMotor.restoreFactoryDefaults();
        shooterMotor.setSmartCurrentLimit(40);
        shooterMotor.setInverted(true);
        shooterMotor.burnFlash();
    }

    /**
     * Sets speed of the shooter (percent)
     */
    public void setSpeed(double percentSpeed) {
        shooterMotor.set(percentSpeed);
    }

    /**
     * Commands the intake send the NOTE up into the shooter
     */
    public void up() {
        intakeMotor.set(Constants.Shooter.kUpPower);
    }

    /**
     * Commands the intake to take in a NOTE from the shooter
     */
    public void down() {
        intakeMotor.set(Constants.Shooter.kDownPower);
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
        return this.cSetSpeed(Constants.Shooter.kShootHighSpeed)
                .andThen(Commands.waitSeconds(Constants.Shooter.shootHighTime))
                .andThen(this.runOnce(this::up))
                .andThen(Commands.waitUntil(() -> false))
                .finallyDo(this::stop);
    }

    /**
     * Spins the shooter up to low speed and then moves the NOTE up into the shooter
     */
    public Command cShootLow() {
        return this.cSetSpeed(Constants.Shooter.kShootLowSpeed)
                .andThen(Commands.waitSeconds(Constants.Shooter.shootLowTime))
                .andThen(this.runOnce(this::up))
                .andThen(Commands.waitUntil(() -> false))
                .finallyDo(this::stop);
    }

    /**
     * Spins the shooter and the intake in reverse to bring a NOTE into position
     */
    public Command cSourceIntake() {
        return this.cSetSpeed(Constants.Shooter.kDownPower)
                .andThen(this.runOnce(this::down))
                .andThen(Commands.waitUntil(() -> false))
                .finallyDo(this::stop);
    }
}
