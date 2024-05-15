package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AnalogInputConstants;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.subsystems.LEDSubsystem.Pattern;

public class GroundIntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    private final AnalogInput irSensor = new AnalogInput(AnalogInputConstants.kIntakeSensor);

    public GroundIntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.CANConstants.kGroundIntakeId, MotorType.kBrushless);

        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setInverted(false);
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Ground Intake/Has Note", hasNote());
        SmartDashboard.putNumber("Ground Intake/Voltage", intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage());
    }

    /**
     * Checks if the ground intake subsystem currently has a note.
     * 
     * @return true if a note is detected, false otherwise.
     */
    public boolean hasNote() {
        return irSensor.getVoltage() > 2.5;
    }

    /**
     * Run the intake at a given power
     * 
     * @return The {@link Command}
     */
    private Command cRunAt(Measure<Dimensionless> power) {
        return runEnd(() -> intakeMotor.set(power.in(Value)), intakeMotor::stopMotor);
    }

    /**
     * Run the intake at low speed
     * 
     * @return The {@link Command}
     */
    public Command cRunLowSpeed() {
        return cRunAt(GroundIntakeConstants.kLowPower);
    }

    /**
     * Run the intake at high speed, <i>stopping</i> when a NOTE is captured
     * 
     * @return The {@link Command}
     */
    public Command cRunUntilCaptured() {
        return Commands.race(
                cRunAt(GroundIntakeConstants.kHighPower),
                Commands.sequence(
                        Commands.waitUntil(this::hasNote).raceWith(LEDSubsystem.cSetOverride(Pattern.CHASING_UP)),
                        LEDSubsystem.cSetOverride(Pattern.FAST_FLASH, 0.15)));
    }

    /**
     * Run the intake at high speed, even if a NOTE is detected.
     * <p>
     * The LEDs will still flash when a note is detected, but the intake will
     * continue to run until the command is interrupted.
     */
    public Command cRun() {
        return Commands.parallel(
                cRunAt(GroundIntakeConstants.kHighPower),
                Commands.waitUntil(this::hasNote).andThen(
                        LEDSubsystem.cSetOverride(Pattern.FAST_FLASH)));
    }
}
