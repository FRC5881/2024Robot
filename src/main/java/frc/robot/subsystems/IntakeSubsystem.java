package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax intakeMotor1;
    private final CANSparkMax intakeMotor2;
    private final DigitalInput intakeStopSensor;

    /* Can IDs */
    public IntakeSubsystem() {
        intakeMotor1 = new CANSparkMax(Constants.IntakeConstants.kIntakeMotor1Port, MotorType.kBrushless);
        intakeMotor2 = new CANSparkMax(Constants.IntakeConstants.kIntakeMotor2Port, MotorType.kBrushless);
        intakeStopSensor = new DigitalInput(1);

    }

    /* Runs intake motors at half speed and stops if a note is inserted */
    public void intake() {
        if (intakeStopSensor.get()) {
            intakeMotor1.set(0);
            intakeMotor2.set(0);
        }

        else {
            intakeMotor1.set(.5);
            intakeMotor2.set(.5);
        }

    }

    /* Stops the intake */
    public void stopIntake() {
        intakeMotor1.set(0);
        intakeMotor2.set(0);
    }

    /* Intake command */
    public Command cIntakeRun() {
        return this.runEnd(this::intake, this::stopIntake);
    }
}
