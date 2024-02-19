package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {

    // DigitalInput lowerInput = new DigitalInput(0);
    DigitalInput upperInput = new DigitalInput(1);

    private final CANSparkMax indexerMotor;

    public IndexerSubsystem() {
        this.indexerMotor = new CANSparkMax(Constants.CANConstants.kIndexerMotor, MotorType.kBrushless);
    }

    /**
     * Moves a note higher through the mechanism
     */
    private void up() {
        indexerMotor.set(Constants.IndexerConstants.kIndexerShooterPower);
    }

    private void down() {
        indexerMotor.set(-Constants.IndexerConstants.kIndexerShooterPower);
    }

    /**
     * Stops the indexer
     */
    private void stop() {
        indexerMotor.set(0);
    }

    /**
     * When a note enters the Indexer it will position it into the indexer
     */
    public Command cPositionNote() {
        return this.run(() -> {
            if (upperInput.get()) {
                this.down();
            } else {
                this.stop();
            }
        });
    }

    // Override the sensor, and send the NOTE to the shooter!
    public Command cSendShooter() {
        return this.runEnd(this::up, this::stop);
    }
}
