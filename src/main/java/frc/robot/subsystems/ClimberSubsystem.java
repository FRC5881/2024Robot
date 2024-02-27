package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

/**
 * ClimberSubsystem controls our climber. It only goes up or down.
 * 
 * Interally we assume that positive voltage is extending and negative voltage
 * is decending.
 */
public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax climberMotor;
    private final DigitalInput limitSwitch = new DigitalInput(0);

    public ClimberSubsystem() {
        climberMotor = new CANSparkMax(Constants.CANConstants.kClimberId, MotorType.kBrushless);
        climberMotor.restoreFactoryDefaults();
        climberMotor.setIdleMode(IdleMode.kBrake);
        climberMotor.setSmartCurrentLimit(30);

        climberMotor.setSoftLimit(SoftLimitDirection.kForward, Constants.ClimberConstants.kForwardLimit);
        climberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber/Height", climberMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("Climber/LimitSwitch", limitSwitch.get());
        SmartDashboard.putNumber("Climber/Voltage", climberMotor.getAppliedOutput() * climberMotor.getBusVoltage());
        SmartDashboard.putNumber("Climber/Current", climberMotor.getOutputCurrent());
        SmartDashboard.putNumber("Climber/Temperature", climberMotor.getMotorTemperature());

        if (limitSwitch.get()) {
            climberMotor.getEncoder().setPosition(0);
        }
    }

    public Command cAutoHome() {
        return run(() -> climberMotor.set(ClimberConstants.kAutoHome.in(Value))).until(limitSwitch::get);
    }

    /**
     * Returns a command that extends the climber while it is ran
     * 
     * @return the Command
     */
    public Command cExtend() {
        return this.startEnd(this::extend, this::stop);
    }

    /**
     * Returns a command that retracts the climber while it is ran
     * 
     * @return the Command
     */
    public Command cRetract() {
        return this.startEnd(this::retract, this::stop);
    }

    private void extend() {
        climberMotor.set(0.5);
    }

    private void retract() {
        climberMotor.set(-1);
    }

    private void stop() {
        climberMotor.stopMotor();
    }
}
