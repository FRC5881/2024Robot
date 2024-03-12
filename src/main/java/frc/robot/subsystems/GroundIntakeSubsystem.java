package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Percent;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GroundIntakeConstants;
import frc.robot.subsystems.LEDSubsystem.Pattern;

/**
 * Ground Intake Subsystem
 * <p>
 * Ground Intake provides 3 operator commands:
 * <ul>
 * <li>Run at high speed, signaling when a note is captured</li>
 * <li>Run at a slower speed, used to help push notes into the shooter</li>
 * <li>Run at high speed, <i>stopping</i> when a note is captured, used for
 * autonomous</li>
 */
public class GroundIntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;

    // True if the sensor is blocked and we've captured a NOTE
    private final DigitalInput irSensor = new DigitalInput(Constants.DIOConstants.kIntakeSensor);

    public GroundIntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.CANConstants.kGroundIntakeId, MotorType.kBrushless);
        intakeMotor.setInverted(false);
    }

    /**
     * Signals the LEDs when a note is captured
     */
    private Command signalAndStop() {
        // TODO: This pattern hasn't been properly choosen

        // Runs the "picking up" pattern until the sensor is blocked
        // Then runs the "picked up" pattern for a short time
        return LEDSubsystem.cSetOverride(Pattern.CHASING_UP, irSensor::get)
                .andThen(LEDSubsystem.cSetOverride(Pattern.FAST_FLASH, 0.5));
    }

    /**
     * Runs the intake at high speed
     */
    private void highSpeed() {
        intakeMotor.set(GroundIntakeConstants.kHighPower.in(Percent));
    }

    /**
     * Runs the intake at low speed
     */
    private void lowSpeed() {
        intakeMotor.set(GroundIntakeConstants.kLowPower.in(Percent));
    }

    /**
     * Stops the intake
     */
    private void stop() {
        intakeMotor.set(0);
    }

    /**
     * Run the intake at high speed, signaling when a note is captured
     * 
     * @return The command to run
     */
    public Command cRunIntake() {
        return runEnd(this::highSpeed, this::stop);
        // .raceWith(signalAndStop().repeatedly());
    }

    /**
     * Run the intake at low speed
     * 
     * @return The command to run
     */
    public Command cRunLowSpeed() {
        return runEnd(this::lowSpeed, this::stop);
    }

    /**
     * Run the intake at high speed, <i>stopping</i> when a note is captured
     * 
     * @return The command to run
     */
    public Command cRunUntilCaptured() {
        return runEnd(this::highSpeed, this::stop).raceWith(signalAndStop());
    }

    /**
     * A command that does nothing but wait for the sensor to be blocked
     */
    public Command waitUntilCaptured() {
        return Commands.waitUntil(irSensor::get);
    }
}
