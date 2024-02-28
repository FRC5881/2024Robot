package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem.Pattern;

/**
 * ClimberSubsystem controls our climber. It only goes up or down.
 * 
 * Interally we assume that positive voltage is extending and negative voltage
 * is decending.
 */
public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax climberMotor;

    public ClimberSubsystem() {
        this.climberMotor = new CANSparkMax(Constants.CANConstants.kClimberId, MotorType.kBrushless);
        this.climberMotor.restoreFactoryDefaults();
        this.climberMotor.setIdleMode(IdleMode.kBrake);

        // climberMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        // climberMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        // climberMotor.setSoftLimit(SoftLimitDirection.kForward,
        // Constants.ClimberConstants.kForwardLimit);
        // climberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    }

    /**
     * Returns a command that extends the climber while it is ran
     * 
     * @return the Command
     */
    public Command cExtend() {
        return this.startEnd(this::extend, this::stop)
                .raceWith(LEDSubsystem.getInstance().cPattern(Pattern.CHASING_UP));
    }

    /**
     * Returns a command that retracts the climber while it is ran
     * 
     * @return the Command
     */
    public Command cRetract() {
        return this.startEnd(this::retract, this::stop)
                .raceWith(LEDSubsystem.getInstance().cPattern(Pattern.CHASING_DOWN));
    }

    private void extend() {
        climberMotor.set(Constants.ClimberConstants.kExtendPower);
    }

    private void retract() {
        climberMotor.set(-Constants.ClimberConstants.kRetractPower);
    }

    private void stop() {
        climberMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber/Height", climberMotor.getEncoder().getPosition());
    }
}
