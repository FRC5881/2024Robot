package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonSRX intakeMotor;
    private final TalonSRX shooterMotor;

    /**
     * adds can id to motors
     */
    public ShooterSubsystem() {
        this.shooterMotor = new TalonSRX(Constants.CANConstants.kShooterId);
        this.intakeMotor = new TalonSRX(Constants.CANConstants.kIntakeId);
    }

    /**
     * sets speed of shooter
     */
    public void setSpeed(double percentSpeed) {
        shooterMotor.set(TalonSRXControlMode.PercentOutput, percentSpeed);
    }

    /**
     * makes the intake send the ball up into the shooter
     */
    public void up() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, Constants.Shooter.kUpPower);
    }

    /**
     * makes the intake send the ball down from the shooter
     */
    public void down() {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, -Constants.Shooter.kDownPower);
    }

    /**
     * turns off all motors
     */
    public void stop() {
        shooterMotor.set(TalonSRXControlMode.PercentOutput, 0);
        intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }

    /**
     * creats a new command that sets the shooter motor speed and then finishes
     */
    private Command cSetSpeed(double percentage) {
        return this.runOnce(() -> {
            this.setSpeed(percentage);
        });
    }

    /**
     * makes the shooter shoot high into the speaker
     */
    public Command cShootHigh() {
        return this.cSetSpeed(Constants.Shooter.kShootHighSpeed)
                .andThen(Commands.waitSeconds(Constants.Shooter.shootHighTime))
                .andThen(this.runOnce(this::up))
                .finallyDo(this::stop);
    }

    /**
     * makes the shooter shoot low into the amp
     */
    public Command cShootLow() {
        return this.cSetSpeed(Constants.Shooter.kShootLowSpeed)
                .andThen(Commands.waitSeconds(Constants.Shooter.shootLowTime))
                .andThen(this.runOnce(this::up))
                .finallyDo(this::stop);
    }

    /**
     * makes the shooter intake a note from the human player
     */
    public Command cShootIntake() {
        return this.cSetSpeed(Constants.Shooter.kDownPower)
                .andThen(Commands.waitSeconds(Constants.Shooter.shootIntakeTime))
                .andThen(this.runOnce(this::down))
                .finallyDo(this::stop);
    }
}
