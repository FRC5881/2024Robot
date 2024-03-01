package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem.Pattern;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor;

    /**
     * sets the can IDs
     */
    public IntakeSubsystem() {
        this.intakeMotor = new CANSparkMax(Constants.CANConstants.kGroundIntakeId, MotorType.kBrushless);
    }

    /**
     * makes the intake suck in a note
     */
    public void suck() {
        intakeMotor.set(Constants.IntakeConstants.kIntakePower);
    }

    /**
     * stops the motor
     */
    public void stop() {
        intakeMotor.stopMotor();
    }

    /**
     * sucks in a note
     */
    public Command cRun() {
        return this.runEnd(this::suck, this::stop)
                .raceWith(LEDSubsystem.getInstance().cPattern(Pattern.SOLID_PURPLE));
    }
}
