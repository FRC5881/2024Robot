package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.LEDSubsystem.Pattern;

/**
 * ClimberSubsystem controls our climber. It only goes up or down.
 * <p>
 * Internally we assume that positive voltage is extending and negative voltage
 * is descending.
 */
public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax climberMotor;

    public ClimberSubsystem() {
        climberMotor = new CANSparkMax(CANConstants.kClimberId, MotorType.kBrushless);

        climberMotor.restoreFactoryDefaults();
        climberMotor.setIdleMode(IdleMode.kBrake);
        climberMotor.setSmartCurrentLimit(40);

        climberMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
        climberMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);

        climberMotor.burnFlash();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber/Height", climberMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Climber/Voltage", climberMotor.getAppliedOutput() * climberMotor.getBusVoltage());
        SmartDashboard.putNumber("Climber/Current", climberMotor.getOutputCurrent());
        SmartDashboard.putNumber("Climber/Temperature", climberMotor.getMotorTemperature());
    }

    /**
     * Returns a command that extends the climber while it is ran
     * 
     * @return the {@link Command}
     */
    public Command cExtend() {
        return startEnd(this::extend, climberMotor::stopMotor)
                .raceWith(LEDSubsystem.cSetOverride(Pattern.CHASING_UP));
    }

    /**
     * Returns a command that retracts the climber while it is ran
     * 
     * @return the {@link Command}
     */
    public Command cRetract() {
        return startEnd(this::retract, climberMotor::stopMotor)
                .raceWith(LEDSubsystem.cSetOverride(Pattern.CHASING_DOWN));
    }

    private void extend() {
        climberMotor.set(ClimberConstants.kExtendPower.in(Value));
    }

    private void retract() {
        climberMotor.set(-ClimberConstants.kRetractPower.in(Value));
    }
}
