package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.LEDSubsystem.Pattern;

/**
 * ClimberSubsystem controls our climber. It only goes up or down.
 * 
 * Internally we assume that positive voltage is extending and negative voltage
 * is descending.
 */
public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax climberMotor;

    public ClimberSubsystem() {
        climberMotor = new CANSparkMax(CANConstants.kClimberId, MotorType.kBrushless);

        climberMotor.restoreFactoryDefaults();
        climberMotor.setIdleMode(IdleMode.kBrake);
        climberMotor.setSmartCurrentLimit(30);

        climberMotor.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.kForwardLimit);
        climberMotor.enableSoftLimit(SoftLimitDirection.kForward, false);

        climberMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
        climberMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);

        climberMotor.burnFlash();

    }

    /**
     * Returns true if the climber is fully retracted
     * 
     * (Uses the hardware limit switch)
     */
    // public boolean isRetracted() {
    // }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber/Height", climberMotor.getEncoder().getPosition());
        // SmartDashboard.putBoolean("Climber/LimitSwitch", isRetracted());
        SmartDashboard.putNumber("Climber/Voltage", climberMotor.getAppliedOutput() * climberMotor.getBusVoltage());
        SmartDashboard.putNumber("Climber/Current", climberMotor.getOutputCurrent());
        SmartDashboard.putNumber("Climber/Temperature", climberMotor.getMotorTemperature());

        // if (isRetracted()) {
        climberMotor.getEncoder().setPosition(0);
        // }
    }

    /**
     * Returns a command that extends the climber while it is ran
     * 
     * @return the Command
     */
    public Command cExtend() {
        return startEnd(this::extend, this::stop).raceWith(LEDSubsystem.cSetOverride(Pattern.CHASING_UP));
    }

    /**
     * Returns a command that retracts the climber while it is ran
     * 
     * @return the Command
     */
    public Command cRetract() {
        return startEnd(this::retract, this::stop).raceWith(LEDSubsystem.cSetOverride(Pattern.CHASING_DOWN));
    }

    public void extend() {
        climberMotor.set(ClimberConstants.kExtendPower.in(Value));
    }

    public void retract() {
        // if (!isRetracted()) {
        climberMotor.set(ClimberConstants.kRetractPower.in(Value));
        // }
    }

    public void stop() {
        climberMotor.stopMotor();
    }

    public boolean isStopped() {
        double speed = climberMotor.getEncoder().getVelocity();
        return MathUtil.isNear(0, speed, .25);
    }

    public void zero() {
        climberMotor.getEncoder().setPosition(0);
        climberMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    }
}
