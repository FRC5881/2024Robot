package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Value;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.AnalogInput;
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

    /**
     * Checks if the ground intake subsystem currently has a note.
     * 
     * @return true if a note is detected, false otherwise.
     */
    private boolean hasNote() {
        return irSensor.getVoltage() < 2.5;
    }

    /**
     * Waits until the ground intake subsystem has captured a note.
     * 
     * @return The {@link Command}
     */
    private Command waitUntilCaptured() {
        return Commands.sequence(
                Commands.race(Commands.waitUntil(this::hasNote), LEDSubsystem.cSetOverride(Pattern.CHASING_UP)),
                Commands.race(Commands.waitSeconds(0.15), LEDSubsystem.cSetOverride(Pattern.FAST_FLASH)));
    }

    /**
     * Run the intake at low speed
     * 
     * @return The {@link Command}
     */
    public Command cRunLowSpeed() {
        return runEnd(() -> intakeMotor.set(GroundIntakeConstants.kLowPower.in(Value)), intakeMotor::stopMotor);
    }

    /**
     * Run the intake at high speed, <i>stopping</i> when a note is captured
     * 
     * @return The {@link Command}
     */
    public Command cRunUntilCaptured() {
        return runEnd(() -> intakeMotor.set(GroundIntakeConstants.kHighPower.in(Value)), intakeMotor::stopMotor)
                .raceWith(waitUntilCaptured());
    }
}
