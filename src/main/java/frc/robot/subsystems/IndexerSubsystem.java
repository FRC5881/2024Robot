package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {

    private final CANSparkMax indexerShooterMotor;

    /**
     * sets the can IDs
     */
    public IndexerSubsystem() {
        this.indexerShooterMotor = new CANSparkMax(Constants.CANConstants.kIndexerShooterMotor, MotorType.kBrushless);
    }

    // 2 analog sensors to detect the NOTE

    private boolean hasNoteBottom() {
        return false;
    }

    private boolean hasNoteTop() {
        return false;
    }

    /**
     * Moves a note higher through the mechanism
     */
    private void up() {
        indexerShooterMotor.set(Constants.IndexerConstants.kIndexerShooterPower);
    }

    private void down() {
        indexerShooterMotor.set(-Constants.IndexerConstants.kIndexerShooterPower);
    }

    /**
     * Stops the indexer
     */
    private void stop() {
        indexerShooterMotor.set(0);
    }

    /**
     * When a note enters the Indexer, it is moved up to the top sensor
     */
    public Command cPositionNote() {
        return this.run(() -> {
            if (hasNoteBottom() && !hasNoteTop()) {
                up();
            } else if (!hasNoteBottom() && hasNoteTop()) {
                down();
            } else {
                stop();
            }
        });
    }

    // Override the sensor, and send the NOTE to the shooter!
    public Command cSendShooter() {
        return this.runEnd(this::up, this::stop);
    }
}
