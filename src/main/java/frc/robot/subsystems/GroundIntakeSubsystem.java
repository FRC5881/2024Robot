package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Value;

import java.sql.Driver;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.units.Dimensionless;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AnalogInputConstants;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.utils.PenningtonLEDs;
import frc.robot.utils.PenningtonLEDs.RawPattern;

public class GroundIntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;
    private final AnalogInput irSensor = new AnalogInput(AnalogInputConstants.kIntakeSensor);
    private final PenningtonLEDs leds = new PenningtonLEDs(0);

    public GroundIntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.CANConstants.kGroundIntakeId, MotorType.kBrushless);

        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setInverted(false);
        intakeMotor.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled()) {
            if (hasNote()) {
                leds.setPattern(RawPattern.FAST_RAINBOW_FLASH);
            } else {
                var color = DriverStation.getAlliance();
                if (color.isPresent()) {
                    if (color.get() == Alliance.Blue) {
                        leds.setPattern(RawPattern.SOLID_BLUE);
                    } else {
                        leds.setPattern(RawPattern.SOLID_RED);
                    }
                } else {
                    leds.setPattern(RawPattern.SOLID_GREEN);
                }
            }
        } else {
            leds.setPattern(RawPattern.SLOW_RAINBOW);
        }

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
                Commands.waitUntil(this::hasNote).andThen(Commands.waitSeconds(0.15)));
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
                Commands.waitUntil(this::hasNote));
    }
}
