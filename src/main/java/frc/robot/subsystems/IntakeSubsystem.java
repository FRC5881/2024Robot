package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax intakeMotor;

    public IntakeSubsystem() {
        this.intakeMotor = new CANSparkMax(Constants.CANConstants.kIntakeId, MotorType.kBrushless);
    }

    public void suck() {
        intakeMotor.set(Constants.Intake.kIntakePower);
    }

    public void stop() {
        intakeMotor.stopMotor();
    }

    public boolean isFull() {
        return false;
    }

    public boolean isEmpty() {
        return !isFull();
    }

    /**
     * sucks while isFull is false
     * 
     * @return
     */
    public Command cRun() {
        return this.runEnd(() -> {
            if (isEmpty()) {
                this.suck();
            } else {
                this.stop();
            }
        }, this::stop);
    }
}
